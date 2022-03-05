
//
//  ft_sensor.h
//  crobot
//
//  Created by Yuechuan Xue on 07/28/2017.
//
//  Copyright (c) 2017 Iowa State University, Robotics Laboratory.
//  http://robotics.cs.iastate.edu/
//  All rights reserved.
//

#ifndef CROBOT_FT_SENSOR_H
#define CROBOT_FT_SENSOR_H
#include <cstdint>
#include <boost/asio.hpp>
#include <Eigen/StdVector>

namespace Eigen
{
	using Vector6d = Matrix<double, 6, 1>;
}

namespace crobot
{
	class AtiFtSensor
	{
	private:
		// There are six commands in the RDT protocol.
		// Any command received by the Net F/T will take
		// precedence over any previously received commands.
		enum class RawDataTransferProtocal
		{
			STOP_STREAMING                   = 0x0000,

			// Real-time response applications
			// Fast (up to 7000 Hz)
			START_HIGH_SPEED_REALTIME_STREAM = 0x0002,

			// Collecting data at high speed, but not responding
			// to it in real-time. Buffer size is set on the
			// Communication Settings web page.
			// Fast (up to 7000 Hz), but comes in bursts (buffers)
			START_HIGH_SPEED_BUFFERED_STREAM = 0x0003,

			// Multi-unit synchronization. The multi-unit ID number
			// is set on the Communication Settings web page.
			// Slower, depending on the number of sensor systems involved
			START_MULTI_UNIT_SYNCH_STREAMING = 0x0004,

			RESET_THRESHOLD_LATCH            = 0x0041,
			SET_SOFTWARE_BIAS                = 0x0042
		};


		struct	RdtRequest
		{
			uint16_t command_header = 0x1234; // Required
			uint16_t command;                 // Command to execute
			uint32_t sample_count;            // Samples to output (0 = infinite)
		} ;

		typedef struct
		{
			uint16_t hdr;          // Always set to 0x1234
			uint16_t cmd;          // The command code, with high bit set to �1�.
			uint32_t count;        // The number of samples to send in response
			uint32_t ipaddr_dest;  // The ip address to send the response to.
			uint16_t port;         // The port to send the response to.
		} ExtendedRdtRequest;

		typedef struct
		{
			uint32_t rdt_sequence;  // RDT sequence number of this packet.
			uint32_t ft_sequence;   // The record�s internal sequence number
			uint32_t status;        // System status code

									// Force and torque readings use counts values
			int32_t fx;             // X-axis force
			int32_t fy;             // Y-axis force
			int32_t fz;             // Z-axis force
			int32_t tx;             // X-axis torque
			int32_t ty;             // Y-axis torque
			int32_t tz;             // Z-axis torque
		} RdtRecord;

		typedef std::unique_ptr<boost::asio::ip::tcp::socket> TcpSocketPtr;
		typedef std::unique_ptr<boost::asio::ip::udp::socket> UdpSocketPtr;
		typedef std::unique_ptr<boost::asio::io_service> IoServicePtr;
		typedef std::unique_ptr<boost::asio::io_context> IoContextPtr;
		typedef std::unique_ptr<boost::asio::ip::udp::resolver::results_type> UdpEndPointsPtr;
		IoServicePtr tcp_io_service_;
		TcpSocketPtr tcp_sock_;
		UdpEndPointsPtr rdt_endpoints_;
		IoContextPtr rdt_io_context_;
		UdpSocketPtr rdt_sock_;

		// TCP Interface
		// The TCP interface listens on TCP port 49151.
		// All commands are 20 bytes in length. All responses
		// begin with the two byte header 0x12, 0x34.

		enum class TcpCommandCodes
		{
			READ_FT         = 0x0000,
			READ_CALIB_INFO = 0x0001,
			WRITE_TRANSFORM = 0x0002,
			WRITE_THRESHOLD = 0x0003
		};

		typedef struct
		{
			uint8_t  command;      // Must be TcpCommandCodes::READ_FT
			uint8_t  reserved[15]; // Should be all 0s
			uint16_t mc_enable;    // Bitmap of MCs to enable
			uint16_t sys_command;  // Bitmap of system command
		} FtCommand;

		typedef struct
		{
			uint16_t header;  // Always 0x1234
			uint16_t status;  // Upper 16bits of status code
			uint16_t fx;
			uint16_t fy;
			uint16_t fz;
			uint16_t tx;
			uint16_t ty;
			uint16_t tz;
		} FtResponse;

		typedef struct
		{
			uint8_t command;       // Must be TcpCommandCodes::READ_CALIB_INFO
			uint8_t reserved[19];  // Should be all 0s
		} CalibInfoCommand;

		typedef struct
		{
			uint16_t header;            // Always 0x1234
			uint8_t  force_units;       // Force units
			uint8_t  torque_units;      // Torque units
			uint32_t counts_per_force;  // Calibration counts per force unit
			uint32_t counts_per_torque; // Calibration counts per torque unit
			uint16_t scale_factors[6];  // Further scaling for 16-bit counts
		} CalibInfoResponse;

		enum class ForceUnit
		{
			POUND       = 0x01,
			NEWTON      = 0x02,
			KILO_POUND  = 0x03,
			KILO_NEWTON = 0x04,
			KILO_GRAM   = 0x05,
			GRAM        = 0x06
		};

		enum class TorqueUnit
		{
			POUND_INCH           = 0x01,
			POUND_FOOT           = 0x02,
			NEWTON_METER         = 0x03,
			NEWTON_MILLIMETER    = 0x04,
			KILO_GRAM_CENTIMETER = 0x05,
			KILO_NEWTON_METER    = 0x06
		};

		typedef struct
		{
			uint8_t command;               // Must be TcpCommandCodes::WRITE_TRANSFORM
			uint8_t transform_dist_units;  // Units of dx, dy, dz
			uint8_t transform_angle_units; // Units of rx, ry, rz
			int16_t transform[6];          // dx, dy, dz, rx, ry, rz
			uint8_t reserved[5];           // Should be all 0s
		} ToolTransformCommand;

		enum class DistanceUnit
		{
			INCH       = 0x01,
			FOOT       = 0x02,
			MILLIMETER = 0x03,
			CENTIMETER = 0x04,
			METER      = 0x05
		};

		enum class AngleUnit
		{
			DEGREES = 0x01,
			RADIANS = 0x02
		};

		typedef struct
		{
			uint8_t command;       // Must be TcpCommandCodes::WRITE_THRESHOLD
			uint8_t index;         // Index of monitor condition. 0-31.
			uint8_t axis;          // 0 = fx, 1 = fy, ..., 5 = tz
			uint8_t output_code;   // Output code of monitor condition.
			int8_t  comparison;    // Comparison code. 1 for "greater than" (>), -1 for "less than" (<).
			int16_t compare_value; // Comparison value, divided by 16 bit scaling factor.
		} MonitorConditionCommand;

		typedef struct
		{
			uint16_t header;       // Always 0x1234
			uint8_t  command_echo; // Echoes command
			uint8_t  status;       // 0 If success, otherwise nonzero
		} WriteResponse;

		CalibInfoResponse calib_info_;
		const uint16_t tcp_port_ = 49151;
		const uint16_t udp_port_ = 49152;
		std::string ipaddr_;

		int32_t bias_[6];
		int32_t rdt_rate_;
		uint32_t cpf_, cpt_;

		RdtRequest rdt_request_;
		bool initialized_ = false;
		bool connected_ = false;
	public:
		using ForceTorqueReading = struct
		{
			double fx, fy, fz;
			double tx, ty, tz;
		};

	public:
		AtiFtSensor() = default;
		~AtiFtSensor();

		int set_ipaddr(const std::string& ipaddr);
		int connect(const std::string& ipaddr);
		int close();

		int start_rdt_stream();
		int rdt_request(Eigen::Vector6d& ft, size_t num) const;
		int rdt_request(Eigen::Vector6d& ft) const;
		int rdt_bias();

		std::string get_info() const;
		int get_ft_reading(Eigen::Vector6d& ft);
		int get_ft_reading(ForceTorqueReading& ft);
	private:
		int read_tcp(void* buf, const size_t length);
		int write_tcp(const void* buf, const size_t length);

		int read_ft(FtResponse& response);
		int read_calib_info(CalibInfoResponse& response);
		int write_transform(const ToolTransformCommand& command);
		int write_threshold(const MonitorConditionCommand& command);
		int initialize();
		int get_settings_from_xml();
	};
}

#endif /* CROBOT_FT_SENSOR_H */