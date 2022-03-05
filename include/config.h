#ifndef CROBOT_CONFIG_H
#define CROBOT_CONFIG_H
#include <string>
#include <vector>
#include "data_struct.h"

class Config
{
public:
	int load(const std::string& filepath);
	std::string to_string() const;
	Link get_upper_link() const;
	Link get_lower_link() const;
	Joint_ get_lower_joint() const;
	Joint_ get_upper_joint() const;

	double lower_link_inertia() const;
	double upper_link_inertia() const;

	struct
	{
		std::string id;
		std::string comment;
		std::string ip_addr;
		std::string ip_addr_static;

		double density;
		double mass;
		double volume;
		double surface_area;
		Eigen::Vector3d mass_center;
		struct
		{
			Eigen::Vector3d ix;
			Eigen::Vector3d iy;
			Eigen::Vector3d iz;
		} principal_axis;
		struct
		{
			double px;
			double py;
			double pz;
		} principal_moment;
		struct
		{
			Eigen::Matrix3d at_mass_center;
			Eigen::Matrix3d at_output;
		} inertia;
		Eigen::Matrix3d orientation;
		Eigen::Vector3d origin;
	} ft_sensor;

	struct
	{
		int cycles;
		double t;
		double dt;
		double g; // gravity acceleration
		Eigen::Vector3d force_limit;
	} expr;

	struct
	{
		double inertia;
		double mass;
		std::vector<double> mass_center;
		std::vector<double> end_point;
	} l1, l2;

	struct
	{
		double kp, ki, kd;
		double alpha;
		double theta;
	} j1, j2;

	struct
	{
		double angle;
		double sx, sy;
	} trajectory;

	struct
	{
		std::vector<double> coeffs;
	} knife;

	struct
	{
		std::string id;
		std::string comment;
		double density;
		double mass;
		double volume;
		double surface_area;
		Eigen::Vector3d mass_center;
		struct
		{
			Eigen::Vector3d ix;
			Eigen::Vector3d iy;
			Eigen::Vector3d iz;
		} principal_axis;
		struct
		{
			double px;
			double py;
			double pz;
		} principal_moment;
		struct
		{
			Eigen::Matrix3d at_mass_center;
			Eigen::Matrix3d at_output;
		} inertia;
		Eigen::Matrix3d orientation;
		Eigen::Vector3d origin;
	} wam[5];

	struct
	{
		bool to_stderr;
		int  buf_secs;
		int  max_size;
		bool is_color;
		std::string dir;
	} log;

	struct
	{
		double theta;
	} fixed_joints;

	struct
	{
		struct
		{
			double kp;
			double ki;
			double kd;
		} y;
		struct
		{
			double kp;
			double ki;
			double kd;
		} angle;
	} pos_control;

	struct
	{
		struct
		{
			double kp;
			double ki;
			double kd;
		} y;
		struct
		{
			double kp;
			double ki;
			double kd;
		} angle;
	} force_control;

	struct
	{
		double kp;
		double ki;
		double kd;
	} hybrid_control;
};

#endif 
