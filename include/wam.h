#ifndef ISU_ROBOTICS_LAB_WAM_H
#define ISU_ROBOTICS_LAB_WAM_H

#include <Eigen/StdVector>
#include <boost/asio.hpp>
#include <thread>

class WamArm
{
public:

	~WamArm();
	int connect();
	int close() const;
	int move(
		double theta1,
		double theta2,
		double theta3,
		double theta4) const;
	int move(
		double theta1,
		double theta2,
		double theta3,
		double theta4,
		bool   block ,
		double max_vel = 0.2,
		double accel   = 1.0) const;
	int move_home() const;

	int set_torque(const Eigen::Vector4d& torque) const;
	int start_torque_control() const;
	int end_torque_control() const;
	int start_position_control() const;
	int end_position_control() const;
	int set_position(const Eigen::Vector4d& pos) const;
	int start_velocity_control() const;
	int end_velocity_control() const;
	int set_velocity(const Eigen::Vector4d& vel) const;

	int request_position(Eigen::Vector4d& position) const;
	int request_velocity(Eigen::Vector4d& velocity) const;
	int request_pos_vel(Eigen::Vector4d& position, Eigen::Vector4d& velocity) const;
	int request_pos_vel_tau(Eigen::Vector4d& position, Eigen::Vector4d& velocity, Eigen::Vector4d& torque) const;
	int request_pos_vel_acc_tau(Eigen::Vector4d& position, Eigen::Vector4d& velocity, Eigen::Vector4d& torque, Eigen::Vector3d& acceleration) const;

	int start_pos_vel_stream();
	int end_pos_vel_stream();
	int update_pos_vel_from_stream(Eigen::Vector4d& position, Eigen::Vector4d& velocity);
private:
	enum class MsgType
	{
		ADVANCED_MOVE           = 0,
		MOVE                    = 1,
		TORQUE_MOVE             = 2,
		BEGIN_TORQUE_CONTROL    = 3,
		END_TORQUE_CONTROL      = 4,
		POS_CONTROL_START       = 5,
		POS_CONTROL_END         = 6,
		POS_CONTROL_MOVE        = 7,
		VEL_CONTROL_START       = 8,
		VEL_CONTROL_END         = 9,
		VEL_CONTROL_MOVE        = 10,
		HOME                    = 11,
		POS_VEL_TAU_REQUEST     = 14,
		POS_REQUEST             = 15,
		VEL_REQUEST             = 16,
		POS_VEL_REQUEST         = 17,
		POS_VEL_STREAM_START    = 18,
		POS_VEL_STREAM_END      = 19,
		POS_VEL_TAU_ACC_REQUEST = 20,
		STOP                    = 100
	};

	typedef std::shared_ptr<boost::asio::ip::tcp::socket> socket_ptr;
	typedef std::shared_ptr<boost::asio::io_service> io_service_ptr;

	typedef struct
	{
		uint16_t type;
		uint16_t length;
	} MsgHeader;

	static size_t make_string_message(const MsgType type, const char* body, char* buffer);
	static size_t make_binary_message(const MsgType type, const void* body, char* buffer, const int length);
	int send_empty_command(const MsgType type) const;
	int write(void* buf, size_t length) const;
	int read(void* buf, size_t length) const;
	static int read(socket_ptr sock, void* buf, size_t length);

	struct WamJointState
	{
		Eigen::Vector4d jp;
		Eigen::Vector4d jv;
		Eigen::Vector4d jt;

		std::mutex mutex;
		bool to_stop;
		bool stop;
		io_service_ptr service;
		socket_ptr sock;
	} joint_state_;
	std::thread wam_background_stream_thread_;
	static void pv_stream_receiver(WamJointState* state);

	socket_ptr     cmd_sock_;
	io_service_ptr cmd_service_;
};

#endif /* ISU_ROBOTICS_LAB_WAM_H */



