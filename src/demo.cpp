// #pragma warning(disable:4996)
// #include "SimpleSerial.h"			 // keep this h file here, if it is relocated after other h files, things will not work out
// #include <thread>
// #define _USE_MATH_DEFINES
// #include <math.h>
// #include <stdlib.h>
// #include <Eigen/LU>
// #include <Eigen/Eigenvalues>
// #include <memory>
// #include <iostream>

// #include "wam.h"
// #include "polynomial.h"
// #include "Functions.h"
// #include "ft_sensor.h"
// #include "KinectDK.h"

// #define FILTER_WINDOW 20
// #define WIN_SIZE 30
// static double theta_dot_history[10][2];
// static double torque_history[FILTER_WINDOW][2];
// static double theta_dot_history_2[WIN_SIZE][2];

// const char* port_name = "\\\\.\\COM4";
// using namespace cv;
// using Vec3 = Eigen::Vector3d;
// using Mat3 = Eigen::Matrix3d;

// void com_read(std::shared_ptr<SimpleSerial> serial)
// {
// 	std::cout << serial->readLine();
// }

// void wam_calibration(WamArm& wam_arm, RigidBody<double>& l1, RigidBody<double>& l2) {
// 	ExprConfig config = {};
// 	expr_initialization(l1, l2, config);

// 	config.init_height = 0.05;
// 	double j1_theta = planning(config, l1);
// 	double j2_theta = config.apcg.const_angle - j1_theta;
// 	int ret = wam_arm.move(0.0, j1_theta, 0.0, j2_theta, true, 0.5, 2.0); if (ret == -1) tear_down();

// 	Polynomial blade = config.lower_coeff;
// 	double c12 = cos(j1_theta + j2_theta);
// 	double s12 = sin(j1_theta + j2_theta);
// 	double c1a = cos(j1_theta + l1.angle);
// 	double s1a = sin(j1_theta + l1.angle);

// 	double bu0 = blade.evaluate(config.u0);
// 	config.lx = -(config.sy - bu0) * s12 + (config.sx - config.u0) * c12 - l1.length * s1a + config.r_wr.x();
// 	std::cout << std::endl;
// 	std::cout << config.lx << std::endl;
// }

// void kinect_calibration(crobot::KinectDK& kinect, RigidBody<double>& l1, RigidBody<double>& l2) {
// 	int tloop = 0;

// 	ExprConfig config = {};
// 	expr_initialization(l1, l2, config);

// 	auto color_mat = cv::Mat();
// 	auto rgbd_mat = cv::Mat();

// 	kinect.get_capture();

// 	while (tloop++ < 5) {
// 		cv::waitKey(100);
// 		int ret = kinect.get_capture();
// 		if (ret != 0) printf("image not available.\n");;
// 		cv::waitKey(30);

// 		if (kinect.get_color_image_from_last_capture(color_mat) == 0)
// 			cv::waitKey(30);
// 		else
// 			printf("color image not available.\n");

// 		if (kinect.get_rgbd_image_from_last_capture(rgbd_mat) == 0)
// 			cv::waitKey(30);
// 		else
// 			printf("rgbd image not available.\n");
// 		image_process_side(config, rgbd_mat);		// decide the initial position of the robot
// 		double j1_theta = planning(config, l1);
// 	}
// }

// int main(int argc, char* argv[])
// {
// 	// setup google log
// 	FLAGS_alsologtostderr = true;
// 	FLAGS_logbufsecs = 0;
// 	FLAGS_max_log_size = 100;
// 	FLAGS_colorlogtostderr = true;
// 	FLAGS_log_dir = "Log";
// 	google::InitGoogleLogging(argv[0]);
	
// 	int ret = 0;

// 	// // define the robot links and joints, initialize the paramenters for the arm
// 	// RigidBody<double> l1, l2;
// 	// Joint_ j1, j2;
// 	// arm_initialization(l1, l2, j1, j2);

// 	// //std::shared_ptr<SimpleSerial> serial = std::make_shared<SimpleSerial>(port_name, 9600);
// 	// //std::thread com_read_thread(com_read, serial);

// 	// // Initialize Barrett WAM arm
// 	// WamArm wam_arm;
// 	// ret = wam_arm.connect();							  if (ret == -1) tear_down();
// 	// ret = wam_arm.move_home();							  if (ret == -1) tear_down();
// 	// ret = wam_arm.move(0., 0., 0., 0.);	                  if (ret == -1) tear_down();
// 	// std::this_thread::sleep_for(std::chrono::seconds(20));
// 	// ret = wam_arm.move_home();                            if (ret == -1) tear_down();

// 	// // setup image caption with KinectDK 
// 	// auto kinect = crobot::KinectDK();
// 	// kinect.connect_default();
// 	// std::this_thread::sleep_for(std::chrono::seconds(1));

// 	// auto color_mat = cv::Mat();
// 	// auto rgbd_mat = cv::Mat();
// 	// cv::waitKey(1000);

// 	//// wam (world frame) calibration 
// 	////wam_calibration(wam_arm, l1, l2);
// 	////ret = wam_arm.move_home();							  if (ret == -1) tear_down();

// 	////camera calibration
// 	///*ret = wam_arm.move(0.0, 1.11, 0.0, 2.66, true, 0.5, 2.0); if (ret == -1) tear_down();
// 	//kinect_calibration(kinect, l1, l2);
// 	//ret = wam_arm.move_home();							  if (ret == -1) tear_down();   */

// 	// // Initialize ATI Delta Force/Torque sensor
// 	// Eigen::Vector6d ft_reading;
// 	// crobot::AtiFtSensor ft_sensor;
// 	// ret = ft_sensor.connect("atinetbox.cs.iastate.edu");  if (ret == -1) tear_down();
// 	// ret = ft_sensor.start_rdt_stream();					  if (ret == -1) tear_down();
// 	// ret = ft_sensor.get_ft_reading(ft_reading);			  if (ret == -1) tear_down();
// 	// LOG(INFO) << "FT Reading: " << ft_reading << std::endl;


// 	//int rounds = 1;		 // for repeat cutting 

// 	// cv::waitKey(1000);
// 	// ret = kinect.get_capture();

// 	// linear guide
// 	//const char* port_name = "\\\\.\\COM4"; //Windows
// 	const char* port_name = "/dev/ttyACM0";  //Linux
// 	std::shared_ptr<SimpleSerial> serial = std::make_shared<SimpleSerial>("/dev/ttyACM0", 9600);
// 	std::thread com_read_thread(com_read, serial);
// 	std::string msg = "s*";
// 	serial->writeString(msg);
// 	std::this_thread::sleep_for(std::chrono::seconds(60));
// 	msg = "e*";
// 	serial->writeString(msg);

// 	return 0;
// }
