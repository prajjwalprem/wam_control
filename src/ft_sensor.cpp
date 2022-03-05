#include "ft_sensor.h"
#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#endif
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <glog/logging.h>

using namespace boost::asio;
using namespace crobot;

AtiFtSensor::~AtiFtSensor()
{
	// release order matters
	tcp_sock_.reset();
	tcp_io_service_.reset();

	rdt_sock_.reset();
	rdt_io_context_.reset();
	rdt_endpoints_.reset();
}


int AtiFtSensor::initialize()
{
	if (!connected_ && !initialized_)
	{
		throw std::runtime_error("ATI F/T Sensor: Connect TCP Before Initialization.");
	}
	get_settings_from_xml();
	read_calib_info(calib_info_);
	initialized_ = true;

	return 0;
}

int AtiFtSensor::connect(const std::string& ipaddr)
{
	if (connected_)
	{
		return -2;
	}

	ipaddr_ = ipaddr;

	using namespace boost::asio;
	try
	{
		tcp_io_service_ = std::make_unique<io_service>();
		ip::tcp::resolver resolver(*tcp_io_service_);
		const ip::tcp::resolver::query query(ipaddr_, std::to_string(tcp_port_));
		const ip::tcp::resolver::iterator iter = resolver.resolve(query);
		ip::tcp::endpoint ep = *iter;

		LOG(INFO) << ep.address().to_string() << ":" << ep.port();

		tcp_sock_ = std::make_unique<ip::tcp::socket>(*tcp_io_service_);
		tcp_sock_->connect(ep);
		const ip::tcp::no_delay nodelay_op(true);
		tcp_sock_->set_option(nodelay_op);

		ip::tcp::socket::receive_buffer_size rbs;
		tcp_sock_->get_option(rbs);
		LOG(INFO) << rbs.value();

		connected_ = true;
		if (!initialized_)
		{
			initialize();
		}
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}

int AtiFtSensor::close()
{
	try
	{
		boost::system::error_code ec;
		tcp_sock_->shutdown(ip::tcp::socket::shutdown_both, ec);
		if (ec) LOG(ERROR) << ec.message();
		tcp_sock_->close(ec);
		if (ec) LOG(ERROR) << ec.message();
	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	connected_ = false;
	return 0;
}

int AtiFtSensor::read_tcp(void* buf, const size_t length)
{
	try
	{
		boost::asio::read(*tcp_sock_, boost::asio::buffer(buf, length));
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}

int AtiFtSensor::write_tcp(const void* buf, const size_t length)
{
	try
	{
		boost::asio::write(*tcp_sock_, boost::asio::buffer(buf, length));
	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}
	return 0;
}


int AtiFtSensor::read_ft(FtResponse& response)
{
	FtCommand cmd;
	cmd.command = uint8_t(TcpCommandCodes::READ_FT);
	memset(cmd.reserved, 0, 15);
	// Each bit position 0-15 in MCEnable corresponds to 
	// the monitor condition at that index.
	// If the bit is a �1�, that monitor condition is enabled.
	// If the bit is a �0�, that monitor condition is disabled.
	cmd.mc_enable = htons(0xFFFF);

	// Bit 0 of sysCommands controls the Bias.
	// If bit 0 is a �1�, the system is biased.
	// If bit 0 is a �0�, no action is taken.
	//
	// Bit 1 of sysCommands controls the monitor condition latch.
	// If bit 1 is a �1�, the monitor condition latch is cleared,
	// and monitor condition evaluation begins again.
	// If bit 1 is a �0�, no action is taken.
	cmd.sys_command = htons(0x0000);

	write_tcp(&cmd, sizeof(cmd));
	read_tcp(&response, sizeof(response));

	return 0;
}

int AtiFtSensor::get_ft_reading(Eigen::Vector6d& ft)
{
	FtResponse reply;
	const auto ret = read_ft(reply);

	assert(ntohs(*reinterpret_cast<int16_t*>(&reply.header)) == 0x1234);
	assert(ntohs(*reinterpret_cast<int16_t*>(&reply.status)) == 0x0000);

	uint16_t* ptr = &reply.fx;
	// index from 0 to 2 represents force readings
	for (int i = 0; i < 3; ++i)
	{
		const int16_t data = ntohs(*reinterpret_cast<int16_t*>(&ptr[i]));
		const int32_t scale_data = data * calib_info_.scale_factors[i];
		ft[i] = double(scale_data) / cpf_;
	}
	// index from 3 to 5 represents torque readings
	for (int i = 3; i < 6; ++i)
	{
		const int16_t data = ntohs(*reinterpret_cast<int16_t*>(&ptr[i]));
		const int32_t scale_data = data * calib_info_.scale_factors[i];
		ft[i] = double(scale_data) / cpt_;
	}

	return ret;
}

int AtiFtSensor::get_ft_reading(ForceTorqueReading& ft)
{
	Eigen::Vector6d reading;
	const auto ret = get_ft_reading(reading);
	if (ret != 0) return ret;

	ft.fx = reading[0];
	ft.fy = reading[1];
	ft.fz = reading[2];
	ft.tx = reading[3];
	ft.ty = reading[4];
	ft.tz = reading[5];

	return ret;
}


int AtiFtSensor::read_calib_info(CalibInfoResponse& response)
{
	CalibInfoCommand cmd;
	cmd.command = uint8_t(TcpCommandCodes::READ_CALIB_INFO);
	memset(cmd.reserved, 0, 19);

	write_tcp(&cmd, sizeof(cmd));
	read_tcp(&response, sizeof(response));

	assert(ntohs(*reinterpret_cast<int16_t*>(&response.header)) == 0x1234);
	response.counts_per_force =
		ntohl(*reinterpret_cast<uint32_t*>(&response.counts_per_force));
	response.counts_per_torque =
		ntohl(*reinterpret_cast<uint32_t*>(&response.counts_per_torque));

	for (unsigned short& scale_factor : response.scale_factors)
	{
		scale_factor = ntohs(*reinterpret_cast<uint16_t*>(&scale_factor));
	}
	return 0;
}

std::string AtiFtSensor::get_info() const
{
	const auto info = calib_info_;
	std::string force_unit;
	switch (info.force_units)
	{
	case uint8_t(ForceUnit::POUND):
		force_unit = "POUND";
		break;
	case uint8_t(ForceUnit::NEWTON):
		force_unit = "NEWTON";
		break;
	case uint8_t(ForceUnit::KILO_POUND):
		force_unit = "KILO POUND";
		break;
	case uint8_t(ForceUnit::KILO_NEWTON):
		force_unit = "KILO NEWTON";
		break;
	case uint8_t(ForceUnit::KILO_GRAM):
		force_unit = "KILO GRAM";
		break;
	case uint8_t(ForceUnit::GRAM):
		force_unit = "GRAM";
		break;

	default:
		LOG(ERROR) << "Code should not come to this line.";
	}

	std::string torque_unit;
	switch (info.torque_units)
	{
	case uint8_t(TorqueUnit::POUND_INCH):
		torque_unit = "POUND_INCH";
		break;
	case uint8_t(TorqueUnit::POUND_FOOT):
		torque_unit = "POUND_FOOT";
		break;
	case uint8_t(TorqueUnit::NEWTON_METER):
		torque_unit = "NEWTON_METER";
		break;
	case uint8_t(TorqueUnit::NEWTON_MILLIMETER):
		torque_unit = "NEWTON_MILLIMETER";
		break;
	case uint8_t(TorqueUnit::KILO_GRAM_CENTIMETER):
		torque_unit = "KILO_GRAM_CENTIMETER";
		break;
	case uint8_t(TorqueUnit::KILO_NEWTON_METER):
		torque_unit = "KILO_NEWTON_METER";
		break;
	default:
		LOG(ERROR) << "Code should not come to this line.";
	}

	return  "Force Unit: " + force_unit + "\tTorque Unit: " + torque_unit;
}


int AtiFtSensor::write_threshold(const MonitorConditionCommand& cmd)
{
	write_tcp(&cmd, sizeof(cmd));
	return 0;
}

int AtiFtSensor::write_transform(const ToolTransformCommand& cmd)
{
	write_tcp(&cmd, sizeof(cmd));
	return 0;
}

int AtiFtSensor::start_rdt_stream()
{
	if (!initialized_)
	{
		throw std::runtime_error("ATI F/T SENSOR: CALL connect BEFORE RDT_STREAM.");
	}

	try
	{
		rdt_io_context_ = std::make_unique<io_context>();
		rdt_sock_ = std::make_unique<ip::udp::socket>(
			*rdt_io_context_, ip::udp::endpoint(ip::udp::v4(), 0));

		ip::udp::resolver resolver(*rdt_io_context_);
		rdt_endpoints_ = std::make_unique<ip::udp::resolver::results_type>(
			resolver.resolve(ip::udp::v4(), ipaddr_, std::to_string(udp_port_)));

	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}

int AtiFtSensor::rdt_request(Eigen::Vector6d& ft) const
{
	return rdt_request(ft, 1);
}

int AtiFtSensor::rdt_request(Eigen::Vector6d& ft, const size_t num) const
{
	*const_cast<uint16_t*>(&rdt_request_.command_header) = htons(0x1234);
	*const_cast<uint16_t*>(&rdt_request_.command) =
		htons(uint16_t(RawDataTransferProtocal::START_HIGH_SPEED_REALTIME_STREAM));
	*const_cast<uint32_t*>(&rdt_request_.sample_count) = htonl(1);

	static uint8_t buf[1024];
	try
	{
		rdt_sock_->send_to(buffer(&rdt_request_, sizeof(RdtRequest)), *(rdt_endpoints_->begin()));
		ip::udp::endpoint sensor_endpoint;
		rdt_sock_->receive_from(buffer(buf, sizeof(RdtRecord)), sensor_endpoint);
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	//RdtRecord reply; // For further use
	//reply.rdt_sequence = ntohl(*reinterpret_cast<uint32_t*>(&buf[0]));
	//reply.ft_sequence  = ntohl(*reinterpret_cast<uint32_t*>(&buf[4]));
	//reply.status       = ntohl(*reinterpret_cast<uint32_t*>(&buf[8]));

	// index from 0 to 2 for torque readings
	for (int i = 0; i < 3; i++) {
		const int32_t data = ntohl(*reinterpret_cast<int32_t*>(&buf[12 + i * 4]));
		ft[i] = double(data) / cpf_;
	}
	// index from 3 to 5 for torque readings
	for (int i = 3; i < 6; i++) {
		const int32_t data = ntohl(*reinterpret_cast<int32_t*>(&buf[12 + i * 4]));
		ft[i] = double(data) / cpt_;
	}

	return 0;
}

int AtiFtSensor::rdt_bias()
{
	*const_cast<uint16_t*>(&rdt_request_.command_header) = htons(0x1234);
	*const_cast<uint16_t*>(&rdt_request_.command) =
		htons(uint16_t(RawDataTransferProtocal::SET_SOFTWARE_BIAS));
	*const_cast<uint32_t*>(&rdt_request_.sample_count) = htonl(0);
	try
	{
		rdt_sock_->send_to(buffer(&rdt_request_, sizeof(RdtRequest)), *(rdt_endpoints_->begin()));
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}


int AtiFtSensor::set_ipaddr(const std::string& ipaddr)
{
	ipaddr_ = ipaddr;
	return 0;
}

// Read elements from XML file
static void find_element_rec(xmlNode * a_node, const std::string& element_to_find, std::string&  ret)
{
	// xmlChar parameter_comp[40];
	for (xmlNode *cur_node = a_node; cur_node; cur_node = cur_node->next)
	{
		if (cur_node->type == XML_ELEMENT_NODE)
		{
			const std::string parameter_comp(reinterpret_cast<const char*>(cur_node->name));

			if (parameter_comp == element_to_find) {
				xmlNode *cur_node_temp = cur_node->children;
				ret = std::string(reinterpret_cast<const char*>(cur_node_temp->content));
				continue;
			}
		}
		find_element_rec(cur_node->children, element_to_find, ret);
	}
}

template<typename T>
static bool get_array_from_string(
	const std::string& str, const char delim, T *data, const size_t len)
{
	size_t start = str.find_first_not_of(delim);
	size_t idx = 0;
	while (start != std::string::npos && idx < len) {
		const size_t end = str.find(delim, start);
		auto token = str.substr(start, end - start);
		if (token.empty())
			token = "0.0";
		double r = stod(token);
		data[idx] = static_cast<T>(r);
		++idx;
		start = str.find_first_not_of(delim, end);
	}
	return (idx == len);
}

int AtiFtSensor::get_settings_from_xml()
{
	std::string filename = "http://" + ipaddr_ + "/netftapi2.xml";

	const auto doc = xmlReadFile(filename.c_str(), nullptr, 0);
	if (doc != nullptr)
	{
		xmlNode *root_element = xmlDocGetRootElement(doc);

		std::string cfgcpf;
		find_element_rec(root_element, "cfgcpf", cfgcpf);
		cpf_ = static_cast<uint32_t>(std::stol(cfgcpf));

		std::string cfgcpt;
		find_element_rec(root_element, "cfgcpt", cfgcpt);
		cpt_ = static_cast<uint32_t>(std::stol(cfgcpt));

		std::string setbias;
		find_element_rec(root_element, "setbias", setbias);
		// 6 tokens separated by semi-colon
		if (!get_array_from_string<int>(setbias, ';', bias_, 6))
		{
			return -1;
		}

		// Read the RDT Output rate
		// xmlChar cfgcomrdtrate[40];
		std::string cfgcomrdtrate;
		find_element_rec(root_element, "comrdtrate", cfgcomrdtrate);
		std::stringstream cfgcomrdtrate_ss;
		cfgcomrdtrate_ss << cfgcomrdtrate;
		cfgcomrdtrate_ss >> rdt_rate_;

		xmlFreeDoc(doc);
		xmlCleanupParser();

		return 0;
	}
	xmlCleanupParser();
	return -1;
}