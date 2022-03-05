#include <iostream>
#include <list>

#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>

#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif
#include <glog/logging.h>
#include "data_struct.h"
#include <thread>
#include <atomic>

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

void  sensorCompensation(Mat3 Rws, Vec3 Fs, Vec3 Ts, Vec3& Fw, Vec3& Tw) {
	static const Vec3 r = { 0.000026416005441,  0.017068920833902, -0.001970453599055 };
	Mat3 rx;
	rx << 0.0, -r[2], r[1],
		r[2], 0.0, -r[0],
		-r[1], r[0], 0.0;

	/*static const Vec3 Fm = { 0.0,  0.0, -7.915 };
	static const Vec3 Fc = { 0.0,  0.0, -7.915 }, Tc = { -0.1209, 0.0, 0.0 };*/
	static const Vec3 Fm = { 0.0,  0.0, -7.7 };
	static const Vec3 Fc = { 0.0,  0.0, -7.7 }, Tc = { -0.113, 0.0, 0.0 };

	Fw = Rws * (Fs - Rws.transpose() * Fm - Fc);
	Tw = -Rws * rx * (Fs - Fc) + Rws * (Ts - Tc);
	return;
}

double calLowestPoint(double th1, double th2, Polynomial blade) {
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

void contourFitting(ExprConfig& config, RigidBody<double> L1, double th1, double th2) {
	std::vector<double> coef;
	//auto points = config.points; 
	//std::shared_ptr<std::vector<Eigen::Vector2d>> points = std::make_shared<std::vector<Eigen::Vector2d>>(config.points); 

	if (config.points.empty())
		return;

	double baseY = config.points[0].y();
	int iPoint = 1;
	for (; iPoint < config.points.size(); iPoint++) {
		if (abs(config.points[iPoint].y() - baseY) < 0.003)
			continue;
		else
			break;
	}
	config.points.erase(config.points.begin(), config.points.begin() + iPoint - 1);

	iPoint = config.points.size() - 1;
	baseY = config.points[iPoint].y();
	for (; iPoint >= 0; iPoint--) {
		if (abs(config.points[iPoint].y() - baseY) < 0.003)
			continue;
		else break;
	}
	config.points.erase(config.points.begin() + iPoint + 2, config.points.begin() + config.points.size());


	/*double u0  = data[index].u0;
	double th1 = data[index].theta[0],  th2 = data[index].theta[1]; */
	double u0 = config.u0;

	double c12 = cos(th1 + th2), s12 = sin(th1 + th2);
	double c1a = cos(th1 + L1.angle), s1a = sin(th1 + L1.angle);

	double maxY = -0.1;
	double cx = 0;
	for (int iPoint = 0; iPoint < config.points.size(); iPoint++) {
		if (config.points[iPoint].y() >= maxY) {
			maxY = config.points[iPoint].y();
			cx = config.points[iPoint].x();
		}
	}

	Polynomial blade = config.lower_coeff;
	auto roots = (blade + Polynomial{ 0, 0, 0, -c12 / s12, -(cx - config.r_wr.x() + L1.length * s1a + config.sy * s12 - config.sx * c12) / s12 }).roots();
	double uc = 0.0;
	for (const auto root : roots)
	{
		if (abs(root.imag() < 1e-6) && root.real() >= 0.0 && root.real() <= 0.15)
		{
			uc = root.real();
			break;
		}
	}

	double cy = (config.sy - blade.evaluate(uc)) * c12 + (config.sx - uc) * s12 + L1.length * c1a + config.r_wr.y();

	if (abs(maxY - cy) > 0.003) {
		double diffY = cy - maxY;
		for (int iPoint = 0; iPoint < config.points.size(); iPoint++) {
			config.points[iPoint][1] += (config.points[iPoint].y() / maxY) * diffY;
		}
	}

	double diffX = (config.points[config.points.size() - 1].x() - config.points[0].x()) / (config.points.size() - 1);
	double xl = config.points[0].x() - 2 * diffX;
	double xr = config.points[config.points.size() - 1].x() + 2 * diffX;

	config.points.push_back({ xl, -0.01 });
	config.points.push_back({ xr, -0.01 });

	double hWeight = 4.0;
	config.points.push_back({ cx, hWeight * cy });

	const int row = config.points.size();
	const int col = 5;

	Eigen::MatrixXd M(row, col);

	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			if (i == row - 1)
				M(i, j) = hWeight * pow(config.points[i].x(), j);
			else
				M(i, j) = pow(config.points[i].x(), j);
		}

	}

	//Eigen::Matrix<double, row, 1, row, 1> Y;
	Eigen::VectorXd Y(row);
	for (int i = 0; i < row; i++)
		Y[i] = config.points[i].y();

	config.points[row - 1][1] /= hWeight;

	Eigen::VectorXd C = (M.transpose() * M).inverse() * M.transpose() * Y;

	for (int i = col - 1; i >= 0; i--)
		coef.push_back(C[i]);

	config.object_coeff = coef;

	// calculate the total cutting area in the cutting plane 
	Polynomial objContour = Polynomial(coef);
	roots = objContour.roots();

	double left = 0.0;
	double right = 0.0;

	for (const auto root : roots)
	{
		if (abs(root.imag() < 1e-6) && abs(root.real() - xl) < abs(4 * diffX))
		{
			left = root.real();
		}

		if (abs(root.imag() < 1e-6) && abs(root.real() - xr) < abs(4 * diffX))
		{
			right = root.real();
		}
	}

	config.leftcp = std::min(left, right);
	config.rightcp = std::max(left, right);

	Polynomial objContourInt = objContour.integral();
	config.totalArea = abs(objContourInt.evaluate(left) - objContourInt.evaluate(right));

	//config.u1 = uc - 0.01; 
	//config.u2 = uc + 0.01; 
	//config.r1 = cx - 0.01; 
	//config.r2 = cx + 0.01; 
	return;
}

bool calculate_contact_point(ExprConfig& config, RigidBody<double> L1, data_t* runtime_data, int index, Eigen::Vector2d velocity, const double th1, const double th2)
{
	static const Polynomial knife_lower_curve = config.lower_coeff;
	static const Polynomial knife_upper_curve = config.upper_coeff;
	static const Polynomial obj_curve = config.object_coeff;

	static const double pox = config.p_obj.x();
	static const double poy = config.p_obj.y();

	const double c12 = cos(th1 + th2);
	const double s12 = sin(th1 + th2);
	const double c1a = cos(th1 + L1.angle);
	const double s1a = sin(th1 + L1.angle);

	static const double sy = config.sy;
	static const double sx = config.sx;
	static const double l1 = L1.length;

	static bool found = false;

	bool inside = false;
	const double pkx = -sy * s12 + sx * c12 - l1 * s1a + config.r_wr.x();
	const double pky = sy * c12 + sx * s12 + l1 * c1a + config.r_wr.y();


	//check whether the knife tip is inside the object 
	if (obj_curve.evaluate(pkx - pox) + poy > pky && pkx > pox) {
		inside = true;
	}

	double u1 = inside ? 0 : config.u1;
	double r1 = inside ? 0 : config.r1;

	//bool first_found = false; 
	int count = 0;
	while (!inside) {
		double bu1 = knife_lower_curve.evaluate(u1);
		//r1 = -(sy - bu1) * s12 + (sx - u1) * c12 - l1 * s1a + config.r_wr.x() - pox;
		double gr1 = obj_curve.evaluate(r1);

		Eigen::Vector2d D1 = { pox + r1 - (-(sy - bu1) * s12 + (sx - u1) * c12 - l1 * s1a + config.r_wr.x()), poy + gr1 - ((sy - bu1) * c12 + (sx - u1) * s12 + l1 * c1a + config.r_wr.y()) };

		if (D1.norm() < 0.00001) {
			if (u1 > 0.0 && u1 < config.u2)
			{
				config.u1 = u1;
				config.r1 = r1;
				//first_found = true; 
				break;
			}
		}

		if (count > 50)
		{
			LOG_FIRST_N(WARNING, 20) << "First: Can't find the contact point of the knife blade and object!";
			return false;
		}

		double dbu1 = knife_lower_curve.derivative().evaluate(u1);
		double dgr1 = obj_curve.derivative().evaluate(r1);

		Eigen::Matrix2d H;
		H.row(0) = Eigen::Vector2d{ 1,     -s12 * dbu1 + c12 };
		H.row(1) = Eigen::Vector2d{ dgr1,   c12 * dbu1 + s12 };
		Eigen::Vector2d cur_val = { r1, u1 };
		Eigen::Vector2d temp = cur_val - H.inverse() * D1;

		r1 = temp[0];
		u1 = temp[1];
		count++;
	}

	double u2 = config.u2;
	double r2 = config.r2;

	if (!found) {
		double u = u1 + 0.005;
		double bu = knife_lower_curve.evaluate(u1);

		double r = -(sy - bu) * s12 + (sx - u) * c12 - l1 * s1a + config.r_wr.x() - pox;
		double y = (sy - bu) * c12 + (sx - u) * s12 + l1 * c1a + config.r_wr.y();

		if (obj_curve.evaluate(r) > y)
		{
			double u_low = u;
			double u_high = 0.2;
			double u_m = 0;
			double r_m = 0;
			while (abs(u_high - u_low) > 0.0005) {
				u_m = (u_low + u_high) / 2;
				double bum = knife_lower_curve.evaluate(u_m);
				r_m = -(sy - bum) * s12 + (sx - u_m) * c12 - l1 * s1a + config.r_wr.x() - pox;

				if (obj_curve.evaluate(r) > (sy - bum) * c12 + (sx - u_m) * s12 + l1 * c1a + config.r_wr.y()) {
					u_low = u_m;
				}
				else
					u_high = u_m;
			}

			u2 = u_m;
			r2 = r_m;
		}
		else {
			double u_low = 0;
			double u_high = u1 - 0.005;
			double u_m = 0;
			double r_m = 0;
			while (abs(u_high - u_low) > 0.0005) {
				u_m = (u_low + u_high) / 2;
				double bum = knife_lower_curve.evaluate(u_m);
				r_m = -(sy - bum) * s12 + (sx - u_m) * c12 - l1 * s1a + config.r_wr.x() - pox;

				if (obj_curve.evaluate(r) > (sy - bum) * c12 + (sx - u_m) * s12 + l1 * c1a + config.r_wr.y()) {
					u_low = u_m;
				}
				else
					u_high = u_m;
			}

			u2 = u_m;
			r2 = r_m;
		}
	}

	count = 0;

	while (1) {
		double bu2 = knife_lower_curve.evaluate(u2);
		//r2 = -(sy - bu2) * s12 + (sx - u2) * c12 - l1 * s1a + config.r_wr.x() - pox;
		double gr2 = obj_curve.evaluate(r2);

		Eigen::Vector2d D2 = { pox + r2 - (-(sy - bu2) * s12 + (sx - u2) * c12 - l1 * s1a + config.r_wr.x()), poy + gr2 - ((sy - bu2) * c12 + (sx - u2) * s12 + l1 * c1a + config.r_wr.y()) };

		if (D2.norm() < 0.00001 && u2 > config.u1) {
			config.u2 = u2;
			config.r2 = r2;
			//second_found = true; 
			break;
		}

		if (count > 50)
		{
			LOG_FIRST_N(WARNING, 20) << "Second: Can't find the contact point of the knife blade and object!";
			return false;
		}

		double dbu2 = knife_lower_curve.derivative().evaluate(u2);
		double dgr2 = obj_curve.derivative().evaluate(r2);

		Eigen::Matrix2d H;
		H.row(0) = Eigen::Vector2d{ 1, -s12 * dbu2 + c12 };
		H.row(1) = Eigen::Vector2d{ dgr2, c12 * dbu2 + s12 };

		Eigen::Vector2d cur_val = { r2, u2 };
		Eigen::Vector2d temp = cur_val - H.inverse() * D2;

		r2 = temp[0];
		u2 = temp[1];
		count++;
	}

	if (config.u1 > config.u2) {
		double temp = config.u1;
		config.u1 = config.u2;
		config.u2 = temp;
		temp = config.r1;
		config.r1 = config.r2;
		config.r2 = temp;
	}

	if (abs(config.u1 - config.u2) < 0.01)
		found = false;
	else
		found = true;

	static const Polynomial klc_int = knife_lower_curve.integral();
	static const Polynomial oc_int = obj_curve.integral();

	double Sa = oc_int.evaluate(r2) - oc_int.evaluate(r1)
		- (sy * c12 + sx * s12 + l1 * c1a + config.r_wr.y()) * (u2 - u1)
		+ c12 * (klc_int.evaluate(u2) - klc_int.evaluate(u1)) + s12 * 0.5 * (u2 * u2 - u1 * u1);

	if (found) {
		runtime_data[index].Sa = Sa;
	}
	else {
		if (index > 0)
			runtime_data[index].Sa = runtime_data[index - 1].Sa;
	}

	return (found);
}

Eigen::Matrix<double, 2, 3> coefficient_integreation(ExprConfig& config, RigidBody<double> L1, data_t* runtime_data, int index, Eigen::Vector2d velocity, const double th1, const double th2,
	Eigen::Vector3d X, double& Ar) {
	static const Polynomial knife_lower_curve = config.lower_coeff;
	static const Polynomial knife_upper_curve = config.upper_coeff;
	static const Polynomial obj_curve = config.object_coeff;

	static const double pox = config.p_obj.x();
	static const double poy = config.p_obj.y();

	const double c12 = cos(th1 + th2);
	const double s12 = sin(th1 + th2);
	const double c1a = cos(th1 + L1.angle);
	const double s1a = sin(th1 + L1.angle);
	const double th1_dot = velocity[0];
	const double th2_dot = velocity[1];

	static const double sy = config.sy;
	static const double sx = config.sx;
	static const double l1 = L1.length;
	static const Eigen::Vector2d vel_board = {config.board_vel, 0 };

	double H11 = 0, H12 = 0, H13 = 0, H21 = 0, H22 = 0, H23 = 0;
	double Len = 0;

	double nu = X[0], Rc = X[1];
	double a = 1 + nu, b = 1 - 2 * nu;

	int segs = 20;
	double step = (config.u2 - config.u1) / segs;

	double bu1 = knife_lower_curve.evaluate(config.u1);
	double x1 = -(sy - bu1) * s12 + (sx - config.u1) * c12 - l1 * s1a;
	double y1 = (sy - bu1) * c12 + (sx - config.u1) * s12 + l1 * c1a;

	double th1_pre = runtime_data[index - 1].theta[1], th2_pre = runtime_data[index - 1].theta[3];
	const double c12_pre = cos(th1_pre + th2_pre);
	const double s12_pre = sin(th1_pre + th2_pre);
	const double c1a_pre = cos(th1_pre + L1.angle);
	const double s1a_pre = sin(th1_pre + L1.angle);
	double dWc = 0.0;

	for (int i = 1; i <= segs; i++) {
		double u = config.u1 + i * step;
		double bu = knife_lower_curve.evaluate(u);
		double x = -(sy - bu) * s12 + (sx - u) * c12 - l1 * s1a;
		double y = (sy - bu) * c12 + (sx - u) * s12 + l1 * c1a;

		double vx = (-(sy - bu) * c12 - (sx - u) * s12) * (th1_dot + th2_dot) - l1 * c1a * th1_dot - vel_board[0];
		double vy = (-(sy - bu) * s12 + (sx - u) * c12) * (th1_dot + th2_dot) - l1 * s1a * th1_dot - vel_board[1];

		double xi = vx / vy;
		double theta = atan((y - y1) / (x - x1));
		double ds = sqrt((y - y1) * (y - y1) + (x - x1) * (x - x1));

		double c1 = -2 * (1 - xi * tan(theta)) * tan(theta);
		double c2 = c1 + xi + tan(theta);
		double c3 = -c1 / tan(theta);
		double c4 = c3 + (xi + tan(theta)) * tan(theta);
		double ratio_d = (c1 * nu + c2) * xi + (c3 * nu + c4);

		double vn = vy * cos(theta) - vx * sin(theta);
		double vt = vx * cos(theta) + vy * sin(theta);

		double xi_tn = vt / vn;
		double r_n = (2 * a + xi_tn * xi_tn);
		double r_d = a + sqrt(a * a + xi_tn * xi_tn / b / b);
		double r = r_n / r_d;
		double cons = r * (cos(theta) - xi * sin(theta)) * ds / ratio_d;

		H12 += (c1 * nu + c2) * cons;
		H22 += (c3 * nu + c4) * cons;

		double pr_pa = 2 / r_d - (1 + a / sqrt(a * a + xi_tn * xi_tn / pow(b, 2))) * r_n / pow(r_d, 2);
		double pr_pb = xi_tn * xi_tn / pow(b, 3) * r_n / sqrt(a * a + xi_tn * xi_tn / pow(b, 2)) / pow(r_d, 2);
		double pr_pnu = pr_pa - 2 * pr_pb;

		H11 += c1 * cons * Rc + pr_pnu * (c1 * nu + c2) * cons * Rc / r - (c1 * nu + c2) * cons * Rc * (c1 * xi + c3) / ratio_d;
		H21 += c3 * cons * Rc + pr_pnu * (c3 * nu + c4) * cons * Rc / r - (c3 * nu + c4) * cons * Rc * (c1 * xi + c3) / ratio_d;

		double x_pre = -(sy - bu) * s12_pre + (sx - u) * c12_pre - l1 * s1a_pre;
		double y_pre = (sy - bu) * c12_pre + (sx - u) * s12_pre + l1 * c1a_pre;

		double dw = sqrt(pow(y - y_pre, 2) + pow(x - x_pre, 2));
		dWc += Rc * r * (cos(theta) - xi * sin(theta)) * ds * dw / sqrt(1 + xi * xi);

		x1 = x; y1 = y;
		Len += ds;
	}

	runtime_data[index].dWc = dWc;

	Eigen::Vector2d v_L1 = { -l1 * c1a * velocity[0], -l1 * s1a * velocity[0] };
	Eigen::Vector2d p_L1 = { -l1 * s1a + config.r_wr.x(),  l1 * c1a + config.r_wr.y() };
	Eigen::Vector2d Ff = { 0.0, 0.0 };

	// calculate frictional force 
	double u = 0;
	double du = step;

	for (int i = 0; i < segs; i++) {
		u = config.u1 + i * du;

		double bu = knife_lower_curve.evaluate(u);
		double wx = -(sy - bu) * s12 + (sx - u) * c12 - l1 * s1a + config.r_wr.x();
		double yl = (sy - bu) * c12 + (sx - u) * s12 + l1 * c1a + config.r_wr.y();

		double dx = du * knife_lower_curve.derivative().evaluate(u) * s12 - du * c12;
		double yh = poy + obj_curve.evaluate(wx - pox);

		double v = 0.0;
		bool   found_root = false;
		auto   poly = knife_upper_curve + Polynomial({ -c12 / s12, -(wx - config.r_wr.x() + l1 * s1a - sx * c12) / s12 - sy });
		std::vector<std::complex<double>> roots;
		poly.roots(roots);
		for (const auto root : roots)
		{
			if (abs(root.imag() < 1e-6) && root.real() >= 0.0 && root.real() <= 0.19)
			{
				v = root.real();
				found_root = true;
				break;
			}
		}

		if (found_root)
		{
			auto cv = knife_upper_curve.evaluate(v);
			double	y_h = (sy - cv) * c12 + (sx - v) * s12 + l1 * c1a + config.r_wr.y();

			yh = yh > y_h ? y_h : yh;
		}

		double dy = (yh - yl) / segs;

		for (int j = 0; j < segs; j++) {
			double y = yl + j * dy;

			Eigen::Vector2d vel = v_L1 + (velocity[0] + velocity[1]) * Eigen::Vector2d{ -(y - p_L1[1]), wx - p_L1[0] } - vel_board;

			vel = vel / vel.norm();
			if (vel[0] != vel[0] || vel[1] != vel[1])    // check whether vel[0] == NAN
				continue;

			Ar += abs(dx * dy);
			Eigen::Vector2d temp_Ff = -2 * vel * dx * dy;
			H13 += temp_Ff[0];
			H23 += temp_Ff[1];
		}
	}


	Eigen::Matrix<double, 2, 3> H;
	//H << H11, H12, H13, H21, H22, H23;
	H.row(0) = Eigen::Vector3d(H11, H12, H13);
	H.row(1) = Eigen::Vector3d(H21, H22, H23);
	return H;
}

Eigen::Vector3d optimization(Eigen::Matrix<double, 2, 3> H, Eigen::Vector3d X, Eigen::Vector3d X_pre, const double fx, const double fy) {
	double lb_nu = 0.01, ub_nu = 0.49;
	double lb_Rc = 0.9 * X_pre[1], ub_Rc = 1.1 * X_pre[1];
	double lb_mP = 0.95 * X_pre[2], ub_mP = 1.05 * X_pre[2];

	if (X_pre[0] > 0 && X_pre[0] < 0.5) {
		lb_nu = std::max(lb_nu, X_pre[0] - 0.02);
		ub_nu = std::min(ub_nu, X_pre[0] + 0.02);
	}
	lb_Rc = std::max(lb_Rc, 0.75 * fy / H(1, 1));
	ub_Rc = std::min(ub_Rc, 0.95 * fy / H(1, 1));
	//std::cout << "matrix H(1,1)= " << H(1, 1) << std::endl; 


	Eigen::Matrix<double, 10, 3> Au;
	Au.row(0) = Eigen::Vector3d(0, H(0, 1), H(0, 2)); 	  Au.row(1) = -Au.row(0);
	Au.row(2) = Eigen::Vector3d(0, H(1, 1), H(1, 2)); 	  Au.row(3) = -Au.row(2);
	Au.row(4) = Eigen::Vector3d(1, 0, 0); 				  Au.row(5) = -Au.row(4);
	Au.row(6) = Eigen::Vector3d(0, 1, 0); 				  Au.row(7) = -Au.row(6);
	Au.row(8) = Eigen::Vector3d(0, 0, 1); 				  Au.row(9) = -Au.row(8);

	double Bu[10] = { 1.1 * fx, -0.9 * fx, 1.1 * fy, -0.9 * fy, ub_nu, lb_nu, ub_Rc, lb_Rc, ub_mP, lb_mP };
	double Weight[10] = { 600, 610, 400, 410, 500, 500, 4, 4.2, 1.0, 1.1 };

	Eigen::Vector3d X_new = X;
	Eigen::Vector3d x_pre = X;

	double Wp = 0, W = 0;
	Eigen::Matrix3d invH = 0.5 * Eigen::Matrix3d::Identity();
	Eigen::Vector3d D;
	for (int i = 0; i < 30; i++) {
		if (i == 0) {
			for (int j = 0; j < 10; j++)
				Wp += Weight[j] * (Au.row(j) * X_new - Bu[j]);
		}

		D = 2 * (X_new - X);

		for (int j = 0; j < 10; j++) {
			D += Weight[j] * Au.row(j);
		}

		X_new -= 0.5 * invH * D;

		if (X_new[0] > ub_nu || X_new[0] < lb_nu)
			X_new[0] = (ub_nu + lb_nu) / 2;
		if (X_new[1] > ub_Rc || X_new[1] < lb_Rc)
			X_new[1] = (ub_Rc + lb_Rc) / 2;
		if (X_new[2] > ub_mP || X_new[2] < lb_mP)
			X_new[2] = (ub_mP + lb_mP) / 2;


		W = (X_new - X).transpose() * (X_new - X);

		for (int j = 0; j < 10; j++) {
			W += Weight[j] * (Au.row(j) * X_new - Bu[j]);
		}

		if (abs(1 - W / Wp) <= 1e-4) {
			break;
		}
		Wp = W;

		/*if ((X_new - x_pre).norm() < 100)
			break;
		x_pre = X_new; */
	}

	if ((X_new - X_pre).norm() > 800)
		X_new = X_pre;

	return X_new;
}

void RLS_estimation(ExprConfig& config, RigidBody<double> L1, data_t* runtime_data, int index, Eigen::Vector2d velocity, double th1, double th2, double fx, double fy) {
	static int count = 0;
	static double sum_x = 0, sum_x2 = 0;
	static double sum_y = 0, sum_y2 = 0;
	static double sum_xy = 0;

	static Eigen::Vector3d X;
	static Eigen::Vector3d X_pre;
	static Eigen::Matrix3d P;
	if (count == 0) {
		if (config.nu < 0.5 && config.nu > 0.3)
			X = Eigen::Vector3d(config.nu, 300, 1000);
		else
			X = Eigen::Vector3d(0.4, 300, 1000);

		P << 0.01, 0, 0, 0, 100, 0, 0, 0, 1000;
	}

	count++;
	double Ar = 0; // knife-object contact area
	Eigen::Matrix<double, 2, 3> H = coefficient_integreation(config, L1, runtime_data, index, velocity, th1, th2, X, Ar);

	Eigen::Vector2d Y = Eigen::Vector2d{ fx, fy };
	Eigen::Matrix<double, 2, 3> H_sub;
	H_sub.row(0) = Eigen::Vector3d(0, H(0, 1), H(0, 2));
	H_sub.row(1) = Eigen::Vector3d(0, H(1, 1), H(1, 2));

	Eigen::Vector2d E = Y - H_sub * X;

	sum_x += E[0];
	sum_x2 += E[0] * E[0];
	sum_y += E[1];
	sum_y2 += E[1] * E[1];
	sum_xy += E[0] * E[1];

	Eigen::Matrix2d R;
	R(0, 0) = sum_x2 / count - pow(sum_x / count, 2);
	R(0, 1) = sum_xy / count - sum_x * sum_y / count / count;
	R(1, 0) = R(0, 1);
	R(1, 1) = sum_y2 / count - pow(sum_y / count, 2);

	if (abs(R(0, 0)) < 0.0001)
		R = Eigen::Matrix2d::Identity();

	Eigen::Matrix<double, 3, 2> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
	P = (Eigen::Matrix3d::Identity() - K * H) * P;

	X_pre = X;
	X += K * E;

	X = optimization(H, X, X_pre, fx, fy);

	//double xi = solve_for_xi(config, L1, X, Ar, th1, th2);


	runtime_data[index].ratio = X[0];
	runtime_data[index].Rc = X[1];
	runtime_data[index].mP = X[2];
	runtime_data[index].xi = 0;

	runtime_data[index].fra_force[0] = H(0, 1) * X[1];
	runtime_data[index].fra_force[1] = H(1, 1) * X[1];
	runtime_data[index].fri_force[0] = H(0, 2) * X[2];
	runtime_data[index].fri_force[1] = H(1, 2) * X[2];
}

