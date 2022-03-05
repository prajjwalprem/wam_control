#include <glog/logging.h>
#include "wam.h"

int WamArm::read(void* buf, const size_t length) const
{
	try
	{
		boost::asio::read(*cmd_sock_, boost::asio::buffer(buf, length));
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}

int WamArm::read(socket_ptr sock, void* buf, size_t length)
{
	try
	{
		boost::asio::read(*sock, boost::asio::buffer(buf, length));
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}


int WamArm::write(void* buf, const size_t length) const
{
	try
	{
		boost::asio::write(*cmd_sock_, boost::asio::buffer(buf, length));
	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}

size_t WamArm::make_string_message(const MsgType type, const char* body, char* buffer)
{
	MsgHeader header;
	header.type = int(type);
	header.length = uint16_t(strlen(body) + 1);

	memcpy(buffer, &header, sizeof(header));
	memcpy(buffer + sizeof(header), body, strlen(body));
	const size_t end = sizeof(header) + strlen(body);
	buffer[sizeof(header) + strlen(body)] = '\0';

	return end + 1;
}

size_t WamArm::make_binary_message(const MsgType type, const void* body, char* buffer, const int length)
{
	MsgHeader header;
	header.type = static_cast<int>(type);
	header.length = length;

	memcpy(buffer, &header, sizeof(header));
	memcpy(buffer + sizeof(header), body, length);

	return length + sizeof(header);
}

int WamArm::send_empty_command(const MsgType type) const
{
	char body[1024], buf[1024];
	snprintf(body, 1024, "");
	const size_t len = make_string_message(type, body, buf);
	const auto numbytes = write(buf, len);
	return numbytes;
}

int WamArm::move(
	double theta1,
	double theta2,
	double theta3,
	double theta4,
	bool   block,
	double max_vel,
	double accel) const
{
	char body[1024], buf[1024];
	const int block_sign = block ? 1 : 0;
	snprintf(body, 1024, "%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %d", theta1, theta2, theta3, theta4, max_vel, accel, block_sign);
	const size_t len = make_string_message(MsgType::ADVANCED_MOVE, body, buf);

	int ret = write(buf, len); if (ret == -1) return ret;
	if (block)
	{
		MsgHeader header;
		ret = read(&header, sizeof(header)); if (ret == -1) return ret;
		ret = read(buf, header.length);      if (ret == -1) return ret;
		printf("%s", buf);
	}

	return 0;
}

int WamArm::move(
	const double theta1,
	const double theta2,
	const double theta3,
	const double theta4) const
{
	char body[1024], buf[1024];
	snprintf(body, 1024, "%.4lf %.4lf %.4lf %.4lf", theta1, theta2, theta3, theta4);
	const size_t len = make_string_message(MsgType::MOVE, body, buf);

	return write(buf, len);
}

int WamArm::move_home() const
{
	return send_empty_command(MsgType::HOME);
}

int WamArm::start_torque_control() const
{
	return send_empty_command(MsgType::BEGIN_TORQUE_CONTROL);
}

int WamArm::end_torque_control() const
{
	return send_empty_command(MsgType::END_TORQUE_CONTROL);
}

int WamArm::set_torque(const Eigen::Vector4d& torque) const
{
	char buffer[256];
	const size_t len = make_binary_message(MsgType::TORQUE_MOVE,
		reinterpret_cast<const char *>(&torque), buffer, sizeof torque);
	write(buffer, len);
	return 0;
}

int WamArm::start_position_control() const
{
	return send_empty_command(MsgType::POS_CONTROL_START);
}

int WamArm::end_position_control() const
{
	return send_empty_command(MsgType::POS_CONTROL_END);
}

int WamArm::set_position(const Eigen::Vector4d& pos) const
{
	char buffer[256];
	const size_t len = make_binary_message(MsgType::POS_CONTROL_MOVE,
		reinterpret_cast<const char *>(&pos), buffer, sizeof pos);
	write(buffer, len);
	return 0;
}

int WamArm::start_velocity_control() const
{
	return send_empty_command(MsgType::VEL_CONTROL_START);
}

int WamArm::end_velocity_control() const
{
	return send_empty_command(MsgType::VEL_CONTROL_END);
}

int WamArm::set_velocity(const Eigen::Vector4d& vel) const
{
	char buffer[256];
	const size_t len = make_binary_message(MsgType::VEL_CONTROL_MOVE,
		reinterpret_cast<const char *>(&vel), buffer, sizeof vel);
	write(buffer, len);
	return 0;
}


int WamArm::request_position(Eigen::Vector4d& position) const
{
	send_empty_command(MsgType::POS_REQUEST);

	char buf[1024];
	MsgHeader header;
	read(&header, sizeof(header));
	read(buf, header.length);
	sscanf(buf, "%lf %lf %lf %lf",
		&position[0], &position[1], &position[2], &position[3]);
	return 0;
}

int WamArm::request_velocity(Eigen::Vector4d& velocity) const
{
	send_empty_command(MsgType::VEL_REQUEST);

	char buf[1024];
	MsgHeader header;
	read(&header, sizeof(header));
	read(buf, header.length);
	sscanf(buf, "%lf %lf %lf %lf",
		&velocity[0], &velocity[1], &velocity[2], &velocity[3]);

	return 0;
}

int WamArm::request_pos_vel(Eigen::Vector4d& position, Eigen::Vector4d& velocity) const
{
	send_empty_command(MsgType::POS_VEL_REQUEST);

	char buf[1024];
	MsgHeader header;
	read(&header, sizeof(header));
	read(buf, header.length);
	sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf",
		&position[0], &position[1], &position[2], &position[3],
		&velocity[0], &velocity[1], &velocity[2], &velocity[3]);

	return 0;
}

int WamArm::request_pos_vel_tau(Eigen::Vector4d& position, Eigen::Vector4d& velocity, Eigen::Vector4d& torque) const
{
	send_empty_command(MsgType::POS_VEL_TAU_REQUEST);

	char buf[1024];
	MsgHeader header;
	read(&header, sizeof(header));
	//	LOG(WARNING) << header.length;
	read(buf, header.length);
	sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		&position[0], &position[1], &position[2], &position[3],
		&velocity[0], &velocity[1], &velocity[2], &velocity[3],
		&torque[0], &torque[1], &torque[2], &torque[3]);

	return 0;
}

int WamArm::request_pos_vel_acc_tau(Eigen::Vector4d& position, Eigen::Vector4d& velocity, Eigen::Vector4d& torque, Eigen::Vector3d& acceleration) const
{
	send_empty_command(MsgType::POS_VEL_TAU_ACC_REQUEST);

	char buf[1024];
	MsgHeader header;
	read(&header, sizeof(header));
	//	LOG(WARNING) << header.length;
	read(buf, header.length);
	sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
		&position[0], &position[1], &position[2], &position[3],
		&velocity[0], &velocity[1], &velocity[2], &velocity[3],
		&torque[0], &torque[1], &torque[2], &torque[3],
		&acceleration[0], &acceleration[1], &acceleration[2]);

	return 0;
}

int WamArm::start_pos_vel_stream()
{
	try
	{
		using namespace boost::asio;
		io_context io_context;
		const auto port = atoi("3300");
		ip::tcp::acceptor a(io_context, ip::tcp::endpoint(ip::tcp::v4(), port));

		joint_state_.to_stop = false;
		joint_state_.stop = false;
		send_empty_command(MsgType::POS_VEL_STREAM_START);
		joint_state_.service = std::make_shared<io_service>();
		joint_state_.sock = std::make_shared<ip::tcp::socket>(*joint_state_.service);
		a.accept(*joint_state_.sock);
		std::thread(pv_stream_receiver, &joint_state_).detach();
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}

int WamArm::end_pos_vel_stream()
{
	send_empty_command(MsgType::POS_VEL_STREAM_END);

	{
		std::lock_guard<std::mutex> lock(joint_state_.mutex);
		joint_state_.to_stop = true;
	}

	const int timeout = 1000;
	for (int counter = 0; counter < timeout; ++counter)
	{
		{
			std::lock_guard<std::mutex> lock(joint_state_.mutex);
			if (joint_state_.stop) return EXIT_SUCCESS;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	try
	{
		joint_state_.sock->close();
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
	}

	return EXIT_FAILURE;
}

int WamArm::update_pos_vel_from_stream(Eigen::Vector4d& position, Eigen::Vector4d& velocity)
{
	std::lock_guard<std::mutex> lock(joint_state_.mutex);
	if (joint_state_.to_stop)
	{
		return -1;
	}

	position = joint_state_.jp;
	velocity = joint_state_.jv;
	return 0;
}

void WamArm::pv_stream_receiver(WamJointState* state)
{
	MsgHeader header = { 0 };
	struct
	{
		Eigen::Vector4d pos;
		Eigen::Vector4d vel;
	} data;
	auto start = std::chrono::system_clock::now();

	boost::asio::ip::tcp::socket::receive_buffer_size rbs;
	state->sock->get_option(rbs);
	size_t receive_buffer_size = rbs.value();
	try
	{
		while (true)
		{
			read(state->sock, &header, sizeof header);
			read(state->sock, &data, header.length);
			{
				std::lock_guard<std::mutex> lock(state->mutex);
				state->jp = data.pos;
				state->jv = data.vel;
				if (state->to_stop) break;
			}

			const auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end - start;
			if (state->sock->available() == receive_buffer_size)
			{
				LOG(WARNING) << "WAMARM STREAM RECEIVE BUFFER FULL.";
			}
			LOG(INFO) << "DATA STREAM DELAY: " << elapsed_seconds.count()
				<< "\tDATA TO READ: " << state->sock->available();
			start = std::chrono::system_clock::now();
		}

		state->sock->close();
	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		return;
	}
	std::lock_guard<std::mutex> lock(state->mutex);
	state->stop = true;
}

WamArm::~WamArm()
{
	cmd_sock_.reset();
	cmd_service_.reset();
}

int WamArm::connect()
{
	using namespace boost::asio;
	try
	{
		cmd_service_ = std::make_shared<io_service>();
		ip::tcp::resolver resolver(*cmd_service_);
		const ip::tcp::resolver::query query("robot-joe.cs.iastate.edu", "3300");
		const ip::tcp::resolver::iterator iter = resolver.resolve(query);
		ip::tcp::endpoint ep = *iter;

		LOG(INFO) << ep.address().to_string() << ":" << ep.port();

		cmd_sock_ = std::make_shared<ip::tcp::socket>(*cmd_service_);
		cmd_sock_->connect(ep);
		const ip::tcp::no_delay nodelay_op(true);
		cmd_sock_->set_option(nodelay_op);

		ip::tcp::socket::receive_buffer_size rbs;
		cmd_sock_->get_option(rbs);
		LOG(INFO) << rbs.value();
	}
	catch (std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}

int WamArm::close() const
{
	try
	{
		cmd_sock_->close();
	}
	catch (const std::exception& e)
	{
		LOG(ERROR) << e.what();
		return -1;
	}

	return 0;
}
