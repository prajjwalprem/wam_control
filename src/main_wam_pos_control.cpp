#pragma warning(disable:4996)
#include "SimpleSerial.h"
#include <thread>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <Eigen/LU>
#include <boost/math/tools/polynomial.hpp>
#include <memory>
#include <iostream>

#include <Eigen/Eigenvalues>
#include "wam.h"
#include "Functions.h"
#include "ft_sensor.h"
#include "KinectDK.h"

#define FILTER_WINDOW 20
static double theta_dot_history[10][2];
static double torque_history[FILTER_WINDOW][2];
static double tz_history[2 * FILTER_WINDOW][2];

const char* port_name = "\\\\.\\COM4";
using namespace cv;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

void com_read(std::shared_ptr<SimpleSerial> serial)
{
	std::cout << serial->readLine();
}
// // Already defined in Functions.cpp line 20 
// void  sensorCompensation(Mat3 Rws, Vec3 Fs, Vec3 Ts, Vec3& Fw, Vec3& Tw) {
// 	static const Vec3 r = { 0.000026416005441,  0.017068920833902, -0.001970453599055 };
// 	Mat3 rx;
// 	rx << 0.0, -r[2], r[1],
// 		r[2], 0.0, -r[0],
// 		-r[1], r[0], 0.0;

// 	/*static const Vec3 Fm = { 0.0,  0.0, -7.915 };
// 	static const Vec3 Fc = { 0.0,  0.0, -7.915 }, Tc = { -0.1209, 0.0, 0.0 };*/
// 	static const Vec3 Fm = { 0.0,  0.0, -7.7 };
// 	static const Vec3 Fc = { 0.0,  0.0, -7.7 }, Tc = { -0.113, 0.0, 0.0 };

// 	Fw = Rws * (Fs - Rws.transpose() * Fm - Fc);
// 	Tw = -Rws * rx * (Fs - Fc) + Rws * (Ts - Tc);
// 	return;
// }

double calContactPoint(double th1, double th2, Polynomial blade) {
	static const double limit = 0.19;

	double dbu0 = -tan(th1 + th2);
	double u0 = -0.0;
	bool found_root = false;

	auto roots = (blade.derivative() + Polynomial({ -dbu0 })).roots();
	for (const auto root : roots)
	{
		if (abs(root.imag() < 1e-6) && root.real() >= -0.002 && root.real() <= limit)
		{
			u0 = root.real();
			found_root = true;
			break;
		}
	}

	return u0;
}

int main(int argc, char* argv[])
{
	// setup google log
	FLAGS_alsologtostderr = true;
	FLAGS_logbufsecs = 0;
	FLAGS_max_log_size = 100;
	FLAGS_colorlogtostderr = true;
	FLAGS_log_dir = "Log";
	google::InitGoogleLogging(argv[0]);

	// define the robot links and joints, initialize the paramenters for the arm
	RigidBody<double> l1, l2;
	Joint_ j1, j2;
	arm_initialization(l1, l2, j1, j2);

	// std::shared_ptr<SimpleSerial> serial = std::make_shared<SimpleSerial>(port_name, 9600);
	// std::thread com_read_thread(com_read, serial);

	// Initialize Barrett WAM arm
	int ret = 0;
	WamArm wam_arm;
	ret = wam_arm.connect();							  if (ret == -1) tear_down();
	ret = wam_arm.move_home();							  if (ret == -1) tear_down();


	// kinect calibration 
  //ret = wam_arm.move(0.0, 1.21076, 0.0, 2.57048, true, 0.5, 2.0); if (ret == -1) tear_down();
	/*int tloop = 0;

	ExprConfig config = {};
	expr_initialization(l1, l2, config);

	config.init_height = 0.0;
	j1.theta = planning(config, l1);
	j2.theta = config.apcg.const_angle - j1.theta;
	//ret = wam_arm.move(0.0, j1.theta, 0.0, j2.theta, true, 0.5, 2.0); if (ret == -1) tear_down();

	Polynomial blade = config.lower_coeff;
	double c12 = cos(j1.theta + j2.theta);
	double s12 = sin(j1.theta + j2.theta);
	double c1a = cos(j1.theta + l1.angle);
	double s1a = sin(j1.theta + l1.angle);

	double bu0 = blade.evaluate(config.u0);
	config.lx = -(config.sy - bu0) * s12 + (config.sx - config.u0) * c12 - l1.length * s1a + config.r_wr.x();
	std::cout << std::endl;
	std::cout << config.lx << std::endl;

	ret = wam_arm.move_home();							  if (ret == -1) tear_down();   */

	// Initialize ATI Delta Force/Torque sensor
	Eigen::Vector6d ft_reading;
	crobot::AtiFtSensor ft_sensor;
	ret = ft_sensor.connect("atinetbox.cs.iastate.edu");  if (ret == -1) tear_down();
	ret = ft_sensor.start_rdt_stream();					  if (ret == -1) tear_down();
	ret = ft_sensor.get_ft_reading(ft_reading);			  if (ret == -1) tear_down();
	LOG(INFO) << "FT Reading: " << ft_reading << std::endl;


	int rounds = 1;
	for (int round = 0; round < rounds; round++) {
		ExprConfig config = {};
		expr_initialization(l1, l2, config);

		// set parameters: initial position and orientation, and the desired constant velocities
		double y0 = 0.08;
		double ang = 0.0*M_PI/180;
		double vel_y = -0.01;
		double vel_x = -0.00;
		double vel_a = -0.0 * M_PI;
		double ang_d = ang;
		double vel_board = 0.0;

		double c12 = cos(ang + M_PI);
		double s12 = sin(ang + M_PI);
		double c1a = (y0 - config.sy * c12 - config.sx * s12 - config.r_wr.y()) / l1.length;
		double s1a = sqrt(1 - c1a * c1a);

		j1.theta = asin(s1a) - l1.angle;
		j2.theta = ang + M_PI - j1.theta;
		Polynomial blade = config.lower_coeff;

		std::cout << "tip x: " << -config.sy * s12 + config.sx * c12 - l1.length * s1a + config.r_wr.x() << std::endl;

		// bias the F/T sensor for once	for repeat cuttings
		if (round == 0) {
			ret = wam_arm.move(0.0, j1.theta, 0.0, M_PI - j1.theta, true, 0.5, 2.0); if (ret == -1) tear_down();
			std::this_thread::sleep_for(std::chrono::seconds(1));

			ret = ft_sensor.rdt_bias();							                     if (ret == -1) tear_down();
			ret = ft_sensor.get_ft_reading(ft_reading);			                     if (ret == -1) tear_down();
			LOG(INFO) << "FT Sensor Unit - " << ft_sensor.get_info() << std::endl;
			LOG(INFO) << "FT Reading: "
				<< std::setprecision(4) << std::fixed << ft_reading[0] << ", "
				<< std::setprecision(4) << std::fixed << ft_reading[1] << ", "
				<< std::setprecision(4) << std::fixed << ft_reading[2] << ", "
				<< std::setprecision(4) << std::fixed << ft_reading[3] << ", "
				<< std::setprecision(4) << std::fixed << ft_reading[4] << ", "
				<< std::setprecision(4) << std::fixed << ft_reading[5];
		}

		// Move to initial desired position
		ret = wam_arm.move(0.0, j1.theta, 0.0, j2.theta, true, 0.5, 2.0);	         if (ret == -1) tear_down();

		int cycles = 650 + 3*500 + 100;//1250 + 400;
		const double dt = 0.002;
		data_t* runtime_data = new data_t[cycles];
		memset(theta_dot_history, 0, sizeof(double) * 10 * 2);
		memset(torque_history, 0, sizeof(double) * FILTER_WINDOW * 2);

		int index_force_ctrl_start = cycles; //touching phase starts
		int index_hybrid_ctrl_start = cycles; // slicing phase starts
		double max_fy = 0.0;
		double initial_px = 0.0;	// slcing phase to record where the contact point is when the slicing phase starts
		bool contourFitted = false;
		bool parametersFitted = false;

		Eigen::Vector4d pos, vel, tau;
		auto start = std::chrono::system_clock::now();

		bool exitloops = false;
		int i = 0;

		ret = wam_arm.start_torque_control();				                         if (ret == -1) tear_down();
		//ret = wam_arm.start_position_control();				                         if (ret == -1) tear_down();

		std::this_thread::sleep_for(std::chrono::seconds(4));
		// std::string msg = "s*";
		// serial->writeString(msg);

		for (i = 0; i < cycles; i++) {
			bool exit_this_loop = false;
			wam_arm.request_pos_vel_tau(pos, vel, tau);

			if (pos[1] < 0.4 || pos[1] > 1.5 || pos[3] < 1.7 || pos[3] > 2.75)
			{
				printf("joint angle out of reasonable range!!!");
				ret = wam_arm.move(0.0, 1.0, 0.0, 2.8, true, 0.5, 2.0);	         if (ret == -1) tear_down();
				exitloops = true;
				break;
			}

			// store history velocity data
			for (size_t j = 0; j < 9; j++)
			{
				theta_dot_history[j][0] = theta_dot_history[j + 1][0];
				theta_dot_history[j][1] = theta_dot_history[j + 1][1];
			}
			theta_dot_history[9][0] = vel[1];
			theta_dot_history[9][1] = vel[3];
			runtime_data[i].theta_dot_raw = Eigen::Vector2d(vel(1), vel(3));

			// linear filter for theta_dot
			const double sigma = 0.0002 / sqrt(2.0 * log(2));
			double filtered_theta_dot[2];
			filtered_theta_dot[0] = velocity_filter(theta_dot_history, 10, 0, sigma, vel(1));
			filtered_theta_dot[1] = velocity_filter(theta_dot_history, 10, 1, sigma, vel(3));

			j1.theta = pos[1];
			j2.theta = pos[3];
			j1.theta_dot = filtered_theta_dot[0];
			j2.theta_dot = filtered_theta_dot[1];

			//dynamic part
			//cos(th2 + alpha+beta)
			double c2ab = cos(j2.theta - j1.alpha + l2.mc_angle);
			double s2ab = sin(j2.theta - j1.alpha + l2.mc_angle);

			const Inertia M11 = l1.mass * l1.mass_center.squaredNorm() + l2.mass * (pow(l1.length, 2) + pow(l2.mc_length, 2) + 2 * l1.length * l2.mc_length * c2ab)
				+ l1.inertia + l2.inertia;
			const Inertia M12 = l2.mass * (l2.mc_length * l2.mc_length + l1.length * l2.mc_length * c2ab) + l2.inertia;
			const Inertia M21 = M12;
			const Inertia M22 = l2.mass * l2.mc_length * l2.mc_length + l2.inertia;

			double C11 = -l2.mass * l1.length * l2.mc_length * s2ab * j2.theta_dot;
			double C12 = -l2.mass * l1.length * l2.mc_length * s2ab * (j1.theta_dot + j2.theta_dot);
			double C21 = l2.mass * l1.length * l2.mc_length * s2ab * j1.theta_dot;
			double C22 = 0.0;

			Eigen::Matrix2d M;
			M << M11, M12, M21, M22;
			Eigen::Matrix2d C;
			C << C11, C12, C21, C22;

			c12 = cos(j1.theta + j2.theta);
			s12 = sin(j1.theta + j2.theta);
			c1a = cos(j1.theta + l1.angle);
			s1a = sin(j1.theta + l1.angle);

			// refers to Jacobian at center of end of second link
			Eigen::Matrix<double, 3, 2> J;
			J.row(0) = Eigen::Vector2d(-l2.end_point.y() * c12 - 0.045 * s12 - l1.length * c1a, -l2.end_point.y() * c12 - 0.045 * s12);
			J.row(1) = Eigen::Vector2d(-l2.end_point.y() * s12 + 0.045 * c12 - l1.length * s1a, -l2.end_point.y() * s12 + 0.045 * c12);
			J.row(2) = Eigen::Vector2d::Ones();

			crobot::AtiFtSensor::ForceTorqueReading ft;
			ft_sensor.get_ft_reading(ft);
			if (abs(ft.fz) > 100.0) {
				ret = wam_arm.end_torque_control();
				LOG(ERROR) << "Force is too large!" << "Index: " << i;
				exitloops = true;
				break;
			}

			// sensor reading to world frame reading, with compensation
			Mat3 Rws;
			Rws << 0, -c12, s12, 1, 0, 0, 0, s12, c12;
			Vec3 Fs = { ft.fx, ft.fy, ft.fz }, Ts = { ft.tx, ft.ty, ft.tz };
			Vec3 Fw, Tw;
			// converts sensor reading from sensor frame back to world frame
			sensorCompensation(Rws, Fs, Ts, Fw, Tw);
			const Vec3 omega = Eigen::Vector3d(-Fw[0], Fw[2], Tw[1]); // force in x and y direction and torque in z-dir
			if (omega[1] > 100)
				break;

			// Calculate contact point (lowest point on the blade polynomial curve)
			config.u0 = calContactPoint(j1.theta, j2.theta, blade);

			if (config.u0 < 0.0) config.u0 = 0.0;

			// y-position of lowest point in the knife frame 
			double bu0 = blade.evaluate(config.u0);
			// position of lowest point in the world frame
			config.lx = -(config.sy - bu0) * s12 + (config.sx - config.u0) * c12 - l1.length * s1a + config.r_wr.x();
			config.ly = (config.sy - bu0) * c12 + (config.sx - config.u0) * s12 + l1.length * c1a + config.r_wr.y();
			// position of a in world frame
			config.ax = -l2.end_point.y() * s12 + 0.045 * c12 - l1.length * s1a + config.r_wr.x();
			config.ay = l2.end_point.y() * c12 + 0.045 * s12 + l1.length * c1a + config.r_wr.y();

			//position of the knife tip in the world frame
			config.px = -config.sy * s12 + config.sx * c12 - l1.length * s1a + config.r_wr.x();
			config.py = config.sy * c12 + config.sx * s12 + l1.length * c1a + config.r_wr.y();
			// y-dir vel of lowest point
			double lp_vely = (-l1.length * s1a) * j1.theta_dot + (-(config.sy - bu0) * s12 + (config.sx - config.u0) * c12) * (j1.theta_dot + j2.theta_dot);

			//std::cout << "lp_y: " << config.ly << ", u0:  " << config.u0 << std::endl; 
			// Initialize position control desired values 
			if (i == 0) {
				config.ypcg.pos = config.py; // desired pos = current value for knife tip
				config.xpcg.pos = config.px;

				config.desired_th1 = j1.theta;
				config.desired_th2 = j2.theta;
			}

			//desired vel -0.08 acc 0.32
			const double acc_desired = 0.0;   // original value 0.16
			// height of the contact board, world frame on cutting board => 0.0
			const double y_contact_plane = 0.000;
			//
			double offset_ly = 0;

			if (config.control == POSITION_CONTROL) {
				// if very close to cutting board
				if (config.ly < y_contact_plane + 0.004) {
					break;
				}
				// switch to touching phase
				if (config.ly + offset_ly < y_contact_plane + 0.001 || config.ly < 0.001) {
					config.fy_min = std::min(config.fy_min, omega[1]);
					if (omega[1] - config.fy_min > 5.0 && !config.contact_board /*&& abs(config.lvy) < 0.005*/) {
						max_fy = omega[1];
						index_force_ctrl_start = i;

						config.control = IMPEDANCE_CONTROL;
						config.ypcg.vel = 0.0;
						config.contact_board = true;

						config.xpcg.acc = 0.0;
						config.xpcg.vel = 0.0;
						config.xpcg.pos = config.ax;
						config.xpcg.int_pos_error = 0.0;
					}
				}
			}

			if (config.control == IMPEDANCE_CONTROL) {
				break;
			}
			runtime_data[i].control_state = int(config.control);

			switch (config.control) {
			case POSITION_CONTROL:
			{
				config.ypcg.vel = vel_y;	//desired vel
				config.ypcg.pos += config.ypcg.vel * dt; //desired position
				//ang_d = ang_d + vel_a * dt;

				config.xpcg.vel = vel_x;	//desired vel
				config.xpcg.pos += config.xpcg.vel * dt;

				/*const double c12_d = cos(ang_d + M_PI);
				const double s12_d = sin(ang_d + M_PI);
				const double c1a_d = (config.ypcg.pos - config.r_wr.y() - config.sy * c12_d - config.sx * s12_d) / l1.length;
				const double s1a_d = sqrt(1 - c1a_d * c1a_d);
				config.desired_th1 = asin(s1a_d) - l1.angle;
				config.desired_th2 = ang_d + M_PI- config.desired_th1;

				const double desired_th1_dot = ( (-config.sy * s12_d + config.sx * c12_d) * vel_a - vel_y) / l1.length / s1a_d;
				const double desired_th1_ddot = -(l1.length * c1a_d * pow(desired_th1_dot, 2) + (config.sy * c12_d + config.sx * s12_d)*vel_a*vel_a) / (l1.length * s1a_d);

				const double desired_px = -config.sy * s12_d + config.sx * c12_d - l1.length * s1a_d + config.r_wr.x();
				const double desired_vx = (-config.sy * c12_d - config.sx * s12_d) * vel_a - l1.length * c1a_d * desired_th1_dot;
				const double desired_ax = (config.sy * s12_d - config.sx * c12_d) * vel_a*vel_a + l1.length * s1a_d * pow(desired_th1_dot, 2) - l1.length * c1a_d * desired_th1_ddot;*/


				/*
				const double c12_d = cos(config.desired_th1 + config.desired_th2);
				const double s12_d = sin(config.desired_th1 + config.desired_th2);
				const double c1a_d = cos(config.desired_th1 + l1.angle);
				const double s1a_d = sin(config.desired_th1 + l1.angle);
				

				const double desired_px = 0;
				const double desired_vx = vel_x;
				const double desired_ax = 0;

				Eigen::Matrix2d Jpd;
				Jpd.row(0) = Eigen::Vector2d(-config.sy * c12_d - config.sx * s12_d - l1.length * c1a_d, -config.sy * c12_d - config.sx * s12_d);
				Jpd.row(1) = Eigen::Vector2d(-config.sy * s12_d + config.sx * c12_d - l1.length * s1a_d, -config.sy * s12_d + config.sx * c12_d);


				Eigen::Vector2d theta_dot = Jpd.inverse() * Eigen::Vector2d{ vel_x, vel_y };

				if (abs(theta_dot[0]) > 0.5 || abs(theta_dot[1]) > 0.5)
					break;
				else {
					config.desired_th1 += theta_dot[0] * dt;
					config.desired_th2 += theta_dot[1] * dt;
				}
				*/


				// velocity for the knife tip
				double vxr = (-config.sy * c12 - config.sx * s12) * (j1.theta_dot + j2.theta_dot) - l1.length * c1a * j1.theta_dot;
				double vyr = (-config.sy * s12 + config.sx * c12) * (j1.theta_dot + j2.theta_dot) - l1.length * s1a * j1.theta_dot;
				config.lvx = vxr;
				config.lvy = vyr;

				// Jacobian at the knife tip
				Eigen::Matrix2d Jp;
				Jp.row(0) = Eigen::Vector2d(-config.sy * c12 - config.sx * s12 - l1.length * c1a, -config.sy * c12 - config.sx * s12);
				Jp.row(1) = Eigen::Vector2d(-config.sy * s12 + config.sx * c12 - l1.length * s1a, -config.sy * s12 + config.sx * c12);


				//double pxe = desired_px - config.px;
				//double vxe = desired_vx - vxr;
				//config.xpcg.int_pos_error += pxe * dt;
				double pxe = config.xpcg.pos - config.px;
				double vxe = config.xpcg.vel - vxr;
				config.xpcg.int_pos_error += pxe * dt;

				double pye = config.ypcg.pos - config.py;
				double vye = config.ypcg.vel - vyr;
				config.ypcg.int_pos_error += pye * dt;

				const Eigen::Vector3d gain_x = { 1000.0, 1200.0 ,  35.0 };
				const Eigen::Vector3d err_x = { pxe,   config.xpcg.int_pos_error, vxe };
				const Eigen::Vector3d gain_y = { 1000.0, 1200.0 ,  35.0 };
				const Eigen::Vector3d err_y = { pye,   config.ypcg.int_pos_error, vye };

				Eigen::Matrix2d M;
				M << M11, M12, M21, M22;
				Eigen::Matrix2d C;
				C << C11, C12, C21, C22;

				Eigen::Vector2d torque = M * Jp.inverse() * Eigen::Vector2d(0 + gain_x.transpose() * err_x, gain_y.transpose() * err_y)
					+ C * Eigen::Vector2d(j1.theta_dot, j2.theta_dot) - J.transpose() * omega;

				j1.torque = torque[0];
				j2.torque = torque[1];

				runtime_data[i].lp_vel = Eigen::Vector2d{ vxr + vel_board, vyr };
				runtime_data[i].lp_pos_desired = Eigen::Vector2d{ desired_px, config.ypcg.pos };
				runtime_data[i].lp_vel_desired = Eigen::Vector2d{ desired_vx, config.ypcg.vel };
				runtime_data[i].lp_acc_desired = Eigen::Vector2d{ desired_ax, config.ypcg.acc };
				runtime_data[i].theta_desired = Eigen::Vector2d{ config.desired_th1, config.desired_th2 };
				runtime_data[i].theta_dot_desired = Eigen::Vector2d{ 0, 0 };
			}
			break;
			}

			runtime_data[i].lp_pos = Eigen::Vector2d{ config.px + i * dt * vel_board, config.py };
			runtime_data[i].theta = pos;
			runtime_data[i].theta_dot = Eigen::Vector4d(0, filtered_theta_dot[0], 0, filtered_theta_dot[1]);
			runtime_data[i].cmd_torque = Eigen::Vector2d(j1.torque, j2.torque);
			runtime_data[i].sensor_force = Eigen::Vector2d(omega[2], Fw[1]);
			runtime_data[i].world_force = Eigen::Vector2d(omega[0], omega[1]);
			runtime_data[i].theta_dot_raw = Eigen::Vector2d(vel[1], vel[3]);
			runtime_data[i].u0 = config.u0;
			runtime_data[i].a_pos = Eigen::Vector2d(config.ax, config.ay);

			if (exit_this_loop)	break;

			// store history torque data
			for (size_t j = 0; j < FILTER_WINDOW - 1; j++)
			{
				torque_history[j][0] = torque_history[j + 1][0];
				torque_history[j][1] = torque_history[j + 1][1];
			}
			torque_history[FILTER_WINDOW - 1][0] = j1.torque;
			torque_history[FILTER_WINDOW - 1][1] = j2.torque;

			// linear filter for sensed torque
			const double sigma_torque_filter = 0.0002 / sqrt(2.0 * log(2));
			double filtered_torque[2];
			filtered_torque[0] = velocity_filter(torque_history, FILTER_WINDOW, 0, sigma_torque_filter, j1.torque);
			filtered_torque[1] = velocity_filter(torque_history, FILTER_WINDOW, 1, sigma_torque_filter, j2.torque);

			// position control to keep other two joints fixed
			Eigen::Vector4d torque;
			torque[0] = (0 - pos[0]) * 10;
			torque[2] = (0 - pos[2]) * 50;

			const bool filter_torque_cd2 = (config.control == HYBRID_CONTROL) && (i - index_hybrid_ctrl_start) * dt < 0.3;
			if (filter_torque_cd2)
			{
				torque[1] = filtered_torque[0];
				torque[3] = filtered_torque[1];
			}
			else
			{
				torque[1] = j1.torque;
				torque[3] = j2.torque;
			}

			runtime_data[i].rd_torque = Eigen::Vector4d(0, filtered_torque[0], 0, filtered_torque[1]);

			ret = wam_arm.set_torque(torque);            if (ret == -1) tear_down();

			/*
			Eigen::Vector4d pos_comd;
			pos_comd[0] = 0.0;
			pos_comd[2] = 0.0;

			pos_comd[1] = config.desired_th1;
			pos_comd[3] = config.desired_th2;

			ret = wam_arm.set_position(pos_comd);            if (ret == -1) tear_down();
			*/


			double time_to_wait;
			do
			{
				auto end = std::chrono::system_clock::now();
				auto elapsed_seconds = end - start;
				time_to_wait = 2000.0 - elapsed_seconds.count() * 1e6;
			} while (time_to_wait > 0.0);
			start = std::chrono::system_clock::now();
		}
		cycles = i;

		ret = wam_arm.end_torque_control();				 if (ret == -1) tear_down();
		//ret = wam_arm.end_position_control();				 if (ret == -1) tear_down();
		ret = wam_arm.move(0.0, 0.4, 0.0, 2.7, true, 0.2, 1.0); if (ret == -1) tear_down();

		// let the linear guide move back 
		// msg = "e*";
		// serial->writeString(msg);


		// save run time data to csv file 
		std::string str = "data" + std::to_string(round) + ".csv";
		const char* name = str.c_str();

		FILE* pFile;
		pFile = fopen(name, "w");
		fprintf(pFile,
			"theta, theta_dot,"
			" p_x, p_y,"
			" p_vx, p_vy,"
			" force_x, force_y, tau_z, force_z \n");
		for (int i = 0; i < cycles - 1; i++)
		{
			fprintf(pFile, "%lf, %lf, ", runtime_data[i].theta(1) + runtime_data[i].theta(3), runtime_data[i].theta_dot(1) + runtime_data[i].theta_dot(3));
			fprintf(pFile, "%lf, %lf, ", runtime_data[i].lp_pos(0), runtime_data[i].lp_pos(1));
			fprintf(pFile, "%lf, %lf, ", runtime_data[i].lp_vel(0), runtime_data[i].lp_vel(1));
			fprintf(pFile, "%lf, %lf, %lf, %lf \n", runtime_data[i].world_force(0), runtime_data[i].world_force(1), runtime_data[i].sensor_force[0], runtime_data[i].sensor_force[1]);
		}
		fclose(pFile);

		//save the fitted curve, cross section area, left and right contact point with the board, and all the obtained points to txt file
		std::ofstream wfile;
		wfile.open("points" + std::to_string(round) + ".txt", std::fstream::in | std::fstream::out | std::fstream::app);

		wfile << "Fitting coeffs: \n";
		for (double cof : config.object_coeff)
			wfile << cof << ",  ";
		wfile << "\n";
		wfile << "TotalArea: " << config.totalArea << "\n";
		wfile << "LeftCp: " << config.leftcp << "\n";
		wfile << "RightCp: " << config.rightcp << "\n";
		wfile << "Points: \n ";
		for (Eigen::Vector2d point : config.points)
			wfile << point[0] << ",  " << point[1] << "\n";

		wfile.close();

		if (exitloops) break;
	}
	ret = wam_arm.move_home();				         if (ret == -1) tear_down();
	ret = wam_arm.close();				             if (ret == -1) tear_down();

	// std::string msg = "e*";
	// serial->writeString(msg);
	// com_read_thread.detach();
	return 0;

}
