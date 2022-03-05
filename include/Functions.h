#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#pragma warning(disable:4996)
#include "SimpleSerial.h"	
#include "data_struct.h"

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>
#endif

#include "polynomial.h"

#include <cstddef>
#include <cstring>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
// Windows data type may not be needed
// typedef const wchar_t* LPCWSTR;
// typedef const char* LPCSTR;
// typedef unsigned char BYTE, *PBYTE, *LPBYTE;
// typedef unsigned short WORD, *PWORD, *LPWORD;
// typedef unsigned long DWORD, *PDWORD, *LPDWORD;
// typedef void* HANDLE;
// typedef struct _DCB {
//   DWORD DCBlength;
//   DWORD BaudRate;
//   DWORD fBinary : 1;
//   DWORD fParity : 1;
//   DWORD fOutxCtsFlow : 1;
//   DWORD fOutxDsrFlow : 1;
//   DWORD fDtrControl : 2;
//   DWORD fDsrSensitivity : 1;
//   DWORD fTXContinueOnXoff : 1;
//   DWORD fOutX : 1;
//   DWORD fInX : 1;
//   DWORD fErrorChar : 1;
//   DWORD fNull : 1;
//   DWORD fRtsControl : 2;
//   DWORD fAbortOnError : 1;
//   DWORD fDummy2 : 17;
//   WORD  wReserved;
//   WORD  XonLim;
//   WORD  XoffLim;
//   BYTE  ByteSize;
//   BYTE  Parity;
//   BYTE  StopBits;
//   char  XonChar;
//   char  XoffChar;
//   char  ErrorChar;
//   char  EofChar;
//   char  EvtChar;
//   WORD  wReserved1;
// } DCB, *LPDCB;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

void    sensorCompensation(Mat3 Rws, Vec3 Fs, Vec3 Ts, Vec3& Fw, Vec3& Tw); 
double  calLowestPoint(double th1, double th2, Polynomial blade);
void    contourFitting(ExprConfig& config, RigidBody<double> L1, double th1, double th2); 

bool    calculate_contact_point(ExprConfig& config, RigidBody<double> L1, data_t* runtime_data, int index, Eigen::Vector2d velocity, const double th1, const double th2);
void    RLS_estimation(ExprConfig& config, RigidBody<double> L1, data_t* runtime_data, int index, Eigen::Vector2d velocity, double th1, double th2, double fx, double fy);
Eigen::Vector3d optimization(Eigen::Matrix<double, 2, 3> H, Eigen::Vector3d X, Eigen::Vector3d X_pre, const double fx, const double fy);
Eigen::Matrix<double, 2, 3> coefficient_integreation(ExprConfig& config, RigidBody<double> L1, data_t* runtime_data, int index, Eigen::Vector2d velocity, const double th1, const double th2,
	Eigen::Vector3d X, double& Ar);

// bool write_com_port(LPCWSTR port_specifier, LPCSTR data)
// {
// 	DCB dcb;
// 	DWORD byteswritten;
// 	HANDLE h_port = CreateFile(
// 		port_specifier,
// 		GENERIC_WRITE,
// 		0,
// 		NULL,
// 		OPEN_EXISTING,
// 		0,
// 		NULL
// 	);

// 	if (!GetCommState(h_port, &dcb))
// 		return false;

// 	dcb.BaudRate = CBR_9600;    // 9600 Baud
// 	dcb.ByteSize = 8;           // 8 data bits
// 	dcb.Parity = NOPARITY;    // no parity
// 	dcb.StopBits = ONESTOPBIT;  // 1 stop
// 	if (!SetCommState(h_port, &dcb))
// 		return false;

// 	bool ret_val = WriteFile(h_port, data, 1, &byteswritten, NULL);
// 	if (!ret_val) std::cout << GetLastError() << std::endl;

// 	CloseHandle(h_port); //close the handle
// 	return ret_val;
// }

static void tear_down()
{
	std::cin.get();
	exit(0);
}

double average(double history[][2], int length, int index)
{
	double sum = 0;
	for (size_t i = 0; i < length; i++)
	{
		sum += history[i][index];
	}

	return sum / length;
}

double velocity_filter(double history[][2], int length, int index, double sigma, double curr_vel)
{
	// double mu[2];
	double mu = average(history, length, index);
	double v = curr_vel;
	double h = exp(-pow(v - mu, 2) / (2.0 * pow(sigma, 2)));
	double p_avg = 1.0 / (1.0 + h);
	double p_new = h / (1.0 + h);
	double v_est = p_new * v + p_avg * mu;

	return v_est;
}

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}


double planning(ExprConfig & config, RigidBody<double> L1) {
	Polynomial blade(config.lower_coeff);
	config.apcg.const_angle = M_PI - atan(blade.derivative().evaluate(config.u0));

	double c12 = cos(config.apcg.const_angle);
	double s12 = sin(config.apcg.const_angle);
	double angle_2 = acos((config.init_height - (config.sy - blade.evaluate(config.u0))*c12 - (config.sx - config.u0)*s12 - config.r_wr.y()) / L1.length);
	if (angle_2 < 0) {
		return M_PI_2 - angle_2 - L1.angle;
	}
	else {
		return  angle_2 - L1.angle;
	}
}

//no need to change if you use the same setup in cutting
void arm_initialization(RigidBody<double> &l1, RigidBody<double> & l2, Joint_ & j1, Joint_ &j2) {
	l1.inertia = 0.204745323759585;
	l1.mass = 5.6772189700;
	l1.mass_center = { 0.000522613744664, 0.119252659363632 }; // !!!!! origin not on the joint
	l1.mc_length = l1.mass_center.norm();
	l1.mc_angle = atan(l1.mass_center[0] / l1.mass_center[1]);

	l1.end_point = { 0.045, 0.55 };
	l1.length = l1.end_point.norm();
	l1.angle = atan(l1.end_point[0] / l1.end_point[1]);

	// link 2 and F/T sensor 
	l2.inertia = 0.055053897627088;
	l2.mass = 2.618875490000000;
	l2.mass_center = { 0.004457227012105, 0.278960054643953 };
	l2.mc_length = sqrt(pow(l2.mass_center[0] - 0.045, 2) + pow(l2.mass_center[1], 2));
	l2.mc_angle = atan((l2.mass_center[0] - 0.045) / l2.mass_center[1]);
	l2.end_point = { -0.045, 0.430 };


	j1.alpha = atan(l1.end_point.x() / l1.end_point.y());
	j2.alpha = atan(l2.mass_center.y() / l2.mass_center.x());

	j1.theta = 0.0;
	j2.theta = 0.0;
	j1.theta_dot = 0;
	j2.theta_dot = 0;

	j1.theta_error_int = 0.0; 
	j2.theta_error_int = 0.0; 
}

void expr_initialization(RigidBody<double> &l1, RigidBody<double> & l2, ExprConfig & config) {

	config.control = POSITION_CONTROL;
	//height of lowest point on knife for initial position and orientation
	config.init_height = 0.050 + 0.002;   //with protector  - 0.003 for knife cutting 

	//lowest point on the knife edge int x-dir in knife frame
	config.u0 = 0.0;
	config.desired_u0 = config.u0;

	// offset from knife tip to the end-effector  (no need to change) 
	config.r_ek[0] = 0.216490 + 0.0518;    	 //rx and ry in the matlab code 
	config.r_ek[1] = -0.01328;

	// offset from the origin of robot frame to world frame  (need to be updated if you change the world frame !!!!!!) 
	//config.r_wr[0] = 0.7036103;    // the old cutting board
	//config.r_wr[1] = 0.413000;	  
	config.r_wr[0] = 0.6226 - 0.06539;		   //the moving cutting board
	config.r_wr[1] = 0.3930 - 0.003; 

	//offset from knife tip to the end of the first link
	config.sx = -l2.end_point.x() + config.r_ek.x();
	config.sy =  l2.end_point.y() - config.r_ek.y();

	//reference point for the object contour fitting in vision in world frame
	config.p_obj[0] = 0.0;
	config.p_obj[1] = 0.0;

	config.u1 = 0.08; // contact points in knife frame on the knife edge
	config.u2 = 0.13;
	config.r1 = 0.00;
	config.r2 = 0.05;
	config.lvx = 0.0; // x-dir velocity for the lowest point in WF
	config.lvy = 0.0;
	config.avx = 0.0; // a is the center of the end of just the second link
	config.avy = 0.0; 

	// not changed
	config.ypcg.Kp = 450;
	config.ypcg.Ki = 55;
	config.ypcg.Kd = 12;

	// initialized here, ypcg : y position control, desired acc and vel of the control point
	config.ypcg.acc = 0.0;
	config.ypcg.vel = 0.0;
	config.ypcg.int_pos_error = 0.0; 

	// fcg : force control
	config.fcg.Kfp = 0.00;
	config.fcg.Kfi = 2.00;
	config.fcg.Kfd = 0.0000020;
	// only for the slicing phase, desired force between knife and cutting board
	config.fcg.fd = 5;
	// force error and integral
	config.fcg.fe = 0;
	config.fcg.int_fe = 0;

	// x-dir position control, never used
	config.xpcg.Kp = 800;
	config.xpcg.Ki = 1600;
	config.xpcg.Kd = 30;

	// desired values
	config.xpcg.pos = 0.0;
	config.xpcg.vel = 0.0;
	config.xpcg.acc = 0.0;
	config.xpcg.aacc = 0.0;
	config.xpcg.int_pos_error = 0.0; 


	// tricky var needed during intial knife-board contact
	config.fy_min = 100.0;
	config.contact_board = false;

	//wrt knife frame
	config.lower_coeff  = { 74 ,  -36.15 ,  7.089 ,-0.744 , 0.0 };
	config.upper_coeff  = { -37.23, 19.7,   -3.82,  0.3265,   0.0 };
	config.object_coeff = { -66.67, 4.0, 0.0 };
	// area of object in the cutting plane
	config.totalArea = 0.0; 
	// x-pos of object contour intersection with cutting board
	config.leftcp    = 0.0;
	config.rightcp   = 0.0;

	Polynomial blade(config.lower_coeff); 
	// constant orientation determined by the lowest point during pressing
	config.apcg.const_angle = M_PI - atan(blade.derivative().evaluate(config.u0));
	// th2 = const_ang - th1

	//
	config.init_height = 0.0; 
	config.init_th1_high = planning(config, l1); 

	config.init_height = 0.05;
	config.init_th1_low = planning(config, l1);

	// used in vision, related to rightCp but little bit larger
	config.lx_max = 0.09; 

	// object parameter fitting
	config.fitting_start_index = 0; 
	config.fitting_end_index = 0; 

	return;
}

// intial position of knife just touching the object
double cal_initialpos(ExprConfig & config, std::vector<Eigen::Vector2d> points) {
	static const Polynomial knife_lower_curve(config.lower_coeff);

	double cangle = config.apcg.const_angle;
	double alpha  = atan(0.045 / 0.55); 
	double l1     = sqrt(0.045 * 0.045 + 0.55*0.55); 

	double low  = config.init_th1_low;  //  1.1784; 
	double high = config.init_th1_high;   //1.3277; 
	double mid  = 0.0; 

	std::vector<double> coef(5);
	coef[0] = config.lower_coeff[0];
	coef[1] = config.lower_coeff[1];
	coef[2] = config.lower_coeff[2];

	//double cp_height = config.init_height;

	while (high > low) {
		mid = (low + high) / 2.0; 
		int count = 0; 
		
		for (Eigen::Vector2d pt : points) {
			coef[3] = -0.744 - 1/tan(cangle); 
			coef[4] = -0.001699 - config.sy + config.sx / tan(cangle) - (l1*sin(mid + alpha) - config.r_wr.x() + pt[0]) / sin(cangle); 

			Polynomial poly = coef; 
			std::vector<std::complex<double>> roots = poly.roots();

			if (roots.empty()) continue; 
			else {
				for (std::complex<double> root : roots) {
					if (abs(root.imag()) < 0.0000001 && root.real() >= 0.0 && root.real() <= 0.18) {
						double bu = knife_lower_curve.evaluate(root.real()); 
						double py = (config.sy - bu)*cos(cangle) + (config.sx - config.u0)*sin(cangle) + l1*cos(mid + alpha) + config.r_wr.y();
						if (py <= pt[1]) {
							count++;
							//cp_height = pt[1];
							break;
						}
					}
				}
			}
			if (count > 2)
				break; 
		}

		if (count > 1)  high = mid; 
		else if (count == 0) low = mid; 
		else {
			//config.init_height = cp_height; 
			return mid;
		}
	}

	return config.init_th1_low;
}

using namespace cv;
void image_process(ExprConfig & config, cv::Mat & image) {

	// // parameter for kinect 2 
	//static const double focal_x = 365.359497;			  
	//static const double focal_y = 365.359497;
	//static const double cx = 256;
	//static const double cy = 212;

	const int cutting_plane_col = 960;	    // need to be recalibrated 
	const int world_frame_row = 120; 
	const int start_row = 0;				// need to be reset
	const int end_row = 110;

	//parameters for k4a 
	static const double focal_x = 918.675;			  
	static const double focal_y = 918.522;
	static const double cx = 960;
	static const double cy = 540;

	int count = 0; 
	int totalval = 0; 
	for(int row = end_row + 1; row < end_row + 10; row++)
		for (int col = cutting_plane_col - 10; col < cutting_plane_col + 10; col++) {
			int cur_val = image.at<uint16_t>(row, col);
			if (cur_val > 705 && cur_val < 740) {			 // for old board, [730,770]
				count++; 
				totalval += cur_val; 
			}
		}

	double 	board_depth = (std::ceil(totalval / count) + 1)/1000; 
	double  shift_x = -(world_frame_row - cy) * board_depth / focal_y;

	std::vector<Eigen::Vector2d> points;
	double max_x = 0.0;
	double max_y = 0.0;

	for (int row = start_row; row <= end_row; ++row)
	{
		const auto z = image.at<uint16_t>(row, cutting_plane_col);
		double py = row;
		double pz = z / 1000.0;

		Eigen::Vector2d point;
		point[0] = (py - cy) * pz / focal_y;
		point[1] = board_depth - pz;

		point[0] = -point[0] - shift_x;


		if (point[1] >= 0.008 && point[1] <= 0.10) {
			//point[1] += 0.02; 
			points.push_back(point);
			max_x = max(max_x, point[0]);
			max_y = max(max_y, point[1]);
		}
	}

		double h_error = 0.005;   //potato 0.8cm, onion 0.5cm, apple 1.2cm, cub 0.6cm

		if (abs(h_error) > 0.003) {
			double h_base = max_y - board_depth;

			for (int i = 0; i < points.size(); i++) {
				points[i][1] += (points[i][1] - board_depth) / h_base * h_error;
			}
			max_y += h_error;
		}


		if (max_x > 0.05)
			config.lx_max = max_x + 0.005;

		if (max_y >= 0.03)
			config.init_height = max_y + 0.005;				// make the knife start above the object

		//double theta_1 = config.init_th1_low;

		if (points.size() != 0 && max_y >= 0.03) {
			//config.object_coeff = contour_fitting(points);
			config.points = points;
			//theta_1 = cal_initialpos(config, points);
		}
		return;
}

// the tripod is on the right side of the board
void image_process_side(ExprConfig& config, cv::Mat& image) {

	//const int cutting_plane_row = 280;	    // need to be recalibrated 
	//const int world_frame_col = 924;

	const int cutting_plane_row = 262;	    // need to be recalibrated 
	const int world_frame_col = 815;		// need to be recalibrated 
	const int start_col = 685;				// need to be reset		   (start_col to end_col should conver the col space of the object) 
	const int end_col = world_frame_col+50;	// no need to change, make sure the object not exceeds this column

	static const double focal_x = 918.675;			//parameters for k4a   
	static const double focal_y = 918.522;
	static const double cx = 960;
	static const double cy = 540;

	int count = 0;
	int totalval = 0;
	for (int col = end_col + 1; col < end_col + 10; col++)
		for (int row = cutting_plane_row - 10; row < cutting_plane_row + 10; row++) {
			int cur_val = image.at<uint16_t>(row, col);
			if (cur_val > 705 && cur_val < 740) {			 // for old board, [730,770], for moving board [705 740], with box [400 430]
				count++;
				totalval += cur_val;
			}
		}

	double 	board_depth = (std::ceil(totalval / count) + 1) / 1000;
	double  shift_x = -(world_frame_col - cx) * board_depth / focal_x;

	std::vector<Eigen::Vector2d> points;
	double max_x = 0.0;
	double max_y = 0.0;

	for (int col = start_col; col <= end_col; ++col)
	{
		const auto z = image.at<uint16_t>(cutting_plane_row, col);
		double px = col;
		double pz = z / 1000.0;

		Eigen::Vector2d point;
		point[0] = (px - cx) * pz / focal_x;
		point[1] = board_depth - pz;

		point[0] = -point[0] - shift_x;


		if (point[1] >= 0.008 && point[1] <= 0.10) {
			//point[1] += 0.02; 
			points.push_back(point);
			max_x = max(max_x, point[0]);
			max_y = max(max_y, point[1]);
		}
	}

	double h_error = 0.007;   //potato 0.8cm, onion 0.5cm, apple 1.2cm, cub 0.6cm

	if (abs(h_error) > 0.003) {
		double h_base = max_y - board_depth;

		for (int i = 0; i < points.size(); i++) {
			points[i][1] += (points[i][1] - board_depth) / h_base * h_error;
		}
		max_y += h_error;
	}


	if (max_x > 0.05)
		config.lx_max = max_x + 0.005;

	if (max_y >= 0.03)
		config.init_height = max_y + 0.005;				// make the knife start above the object

	//double theta_1 = config.init_th1_low;

	if (points.size() != 0 && max_y >= 0.03) {
		//config.object_coeff = contour_fitting(points);
		config.points = points;
		//theta_1 = cal_initialpos(config, points);
	}

		/*for (int i = 0; i < points.size(); i++) {
			std::cout << points[i].x() << "," << points[i].y() << std::endl;
		}*/
	return;
}

//std::vector<double> contour_fitting(std::vector<Eigen::Vector2d> points) {
//	std::vector<double> coef;
//
//	if (points.empty())
//		return coef;
//
//	const int row = points.size();
//	const int col = 5;   //we will fit a quatic curve 
//						 //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M;
//	Eigen::MatrixXd M(row, col);
//	//Eigen::Vector3d x = A.ldlt().solve(Eigen::VectorXd::Zero(row));
//
//	for (int i = 0; i < row; i++) {
//		for (int j = 0; j < col; j++)
//			M(i, j) = pow(points[i].x(), j);
//	}
//
//	//Eigen::Matrix<double, row, 1, row, 1> Y;
//	Eigen::VectorXd Y(row);
//	for (int i = 0; i < row; i++)
//		Y[i] = points[i].y();
//
//	Eigen::VectorXd C = (M.transpose() * M).inverse() * M.transpose() * Y;
//
//	for (int i = col - 1; i >= 0; i--)
//		coef.push_back(C[i]);
//
//	return coef;
//}

#endif 


