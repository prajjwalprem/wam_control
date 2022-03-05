#ifndef CROBOT_DATA_STRUCT_H
#define CROBOT_DATA_STRUCT_H

#include <iostream>
#include <fstream>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include "polynomial.h"

using Force3D               = Eigen::Vector3d;
using Displacement3D        = Eigen::Vector3d;
using Acceleration3D        = Eigen::Vector3d;
using AngularVelocity3D     = Eigen::Vector3d;
using Velocity3D            = Eigen::Vector3d;
using AngularAcceleration3D = Eigen::Vector3d;
using Torque3D              = Eigen::Vector3d;

using Force2D               = Eigen::Vector2d;
using Displacement2D        = Eigen::Vector2d;
using Acceleration2D        = Eigen::Vector2d;
using AngularVelocity2D     = Eigen::Vector2d;
using Velocity2D            = Eigen::Vector2d;
using AngularAcceleration2D = Eigen::Vector2d;
using Torque2D              = Eigen::Vector2d;
using Mass                  = double;
using Angle                 = double;
using Inertia               = double;
using Length                = double;
using Tmp                   = double;
using Torque                = double;
using AngularVelocity       = double;
using Acceleration          = double;
using AngularAcceleration   = double;


template< typename T >
class RigidBody
{
public:
	Displacement2D mass_center;
	Length         mc_length;
	Angle          mc_angle; 
	Displacement2D origin;
	Inertia        inertia;
	Mass           mass;
	Angle          angle;
	Displacement2D end_point;
	Length         length;
};

typedef RigidBody<double> Link;

using RobotConfig = struct RobotConfig
{
	struct Joint
	{
		Angle			angle;
		Torque2D		torque;
		AngularVelocity velocity;
	} j1, j2;
};

using Pid = struct Pid
{
	double kp;
	double ki;
	double kd;
};

using Joint_ = struct Joint_
{
	Angle theta;
	Angle alpha;
	Torque torque;
	AngularVelocity theta_dot;
	AngularAcceleration theta_ddot;
	Pid pid;

	Angle theta_error_int; 
};

using Segment = struct Segment{
	Angle th1; 
	Angle th2 ; 
	double rc ; 
	bool  bound_set; 
};

using ControlMethod = enum {
	POSITION_CONTROL  = 0,
	FORCE_CONTROL     = 1,
	HYBRID_CONTROL    = 2,
	IMPEDANCE_CONTROL = 3
};


using Pos_Control_gains = struct Pos_Control_gains {
	double     pos; 
	double     vel; 
	double     acc;
	double     aacc;
	double     const_angle = 0.0; 

	double     int_pos_error; 
	double     int_angle_error; 

	double     Kp;
	double     Ki;
	double     Kd;
};

using Force_contorl_gains = struct Force_control_gains {
	double     fd; 
	double     fe;
	double     int_fe;
	double     d_fe;
	double     f_model_y;

	double     Kfp = 0;
	double     Kfd = 0;
	double     Kfi = 0;
};

using CuttingObject = enum {
	POTATO, 
	ONION,
	EGGPLANT
};

using Obj_parameters = struct Obj_parameters {
	double     R;    //fracture tuffness 
	double     P;   //pressure from side 
	double     miu; //friction coefficient 
};

using LS_fitting = struct LS_fitting {
	double     x1x1; 
	double     x1x2; 
	double     x2x2; 
	double     yx1; 
	double     yx2; 
	double     miuP; 
	double     R; 
};


using ExprConfig = struct ExprConfig
{
	Displacement2D r_ek;
	Displacement2D r_wr; 
	Displacement2D p_obj;
	
	double         sy; 
	double         sx; 

	// variables for motion planning 
	double         init_height;
	double         lx_max;   //the maximum x iodinate when the knife should stop slicing
	double         u0; 
	double         lx;		    //lowest point x coordinate 
	double         ly;			//lowest point y coordinate 
	double         lvx;			//lowest point velocity in x_direction 
	double         lvy;         //lowest point velocity in y_direction 
	double         desired_u0; 
	double         desired_th1; 
	double         desired_th2; 
	double         ax; 
	double         ay;
	double         avx; 
	double         avy; 
	double  px; 	   // knife tip point
	double  py; 
	double  pvx; 
	double  pvy; 

	//variables for knife-boject contact points in the knife frame
	double         u1; 			 //edge left contact 
	double         u2; 			 //edge right contact 
	double         r1; 			 //spine left contact 
	double         r2; 			 //spine right contact
	double         nu;			 // object's Poisson's ratio


	Pos_Control_gains   apcg; 
	Pos_Control_gains   ypcg; 
	Pos_Control_gains   xpcg;
	Force_contorl_gains fcg; 
	LS_fitting          lsf; 

	CuttingObject  object = POTATO; 
	Obj_parameters obj_prameters; 

	//task state variables 
	bool           contact_object;
	bool           contact_board;
	double         fy_min; 
	double         board_vel;   // velocity of the cutting board along the negtive x-direction 

	ControlMethod  control = POSITION_CONTROL; 

	//knife fitting coefficients blade and upper edge 
	std::vector<double>  lower_coeff;
	std::vector<double>  upper_coeff;
	std::vector<double>  object_coeff;		  //for saving object coutour coefficients
	double totalArea; 	 //cross section area of the object with the cutting plane
	double leftcp; 		 // left contact point of the object with the cutting board 
	double rightcp; 	 // right contact point

	double init_th1_low; 
	double init_th1_high; 
	std::vector<Eigen::Vector2d> points;	  //for saving contour points

	int fitting_start_index; 
	int fitting_end_index; 
};

typedef struct
{
	Eigen::Vector2d cmd_torque;
	Eigen::Vector4d rd_torque;

	Eigen::Vector4d theta;
	Eigen::Vector4d theta_dot;
	Eigen::Vector2d theta_dot_raw;

	Eigen::Vector3d fri_force;
	Eigen::Vector3d fra_force;
	Eigen::Vector2d sensor_force;
	Eigen::Vector2d world_force; 

	Eigen::Vector2d a_pos; 
	Eigen::Vector2d lp_pos;
	Eigen::Vector2d lp_vel;
	double          u0; 
	double          ratio; //uncut area over total area
	double          Rc; 
	double          mP; 
	double          xi; 
	double          Sa; 
	double          dWc; 

	Eigen::Vector2d lp_pos_desired; 
	Eigen::Vector2d lp_vel_desired; 
	Eigen::Vector2d lp_acc_desired; 

	Eigen::Vector2d theta_desired;
	Eigen::Vector2d theta_dot_desired;
	Eigen::Vector2d theta_ddot_desired;

	Eigen::Vector4d time_delay;
	int             control_state; 

	Eigen::Vector3d ft_comp; 
} data_t;


	class Devector {
	public:
		void add_loc(double num);
		double mean_loc();
		double deviation_loc();
		bool is_fill(); 

	private:
		std::vector<double>  data;
		int pos = -1;
		bool data_fill = false;

		bool mean_update = false;
		double _mean = 0.0;
		double _dev = 0.0;
	};

#endif
