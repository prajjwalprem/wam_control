#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H
#include <Eigen/Eigen>

class Polynomial
{
	// Coefficients from high to low 
	std::vector<double> coeffs;
	Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;

public:
	Polynomial(std::initializer_list<double> l)
	{
		coeffs.insert(coeffs.end(), l.begin(), l.end());
	}

	Polynomial(std::vector<double> l)
	{
		coeffs = l;
	}

	double evaluate(double x) const
	{
		size_t order = coeffs.size() - 1;
		double result = 0.0;
		for (int i = 0; i < coeffs.size(); i++) {
			result += coeffs[i] * pow(x, order - i);
		}

		return result;
	}

	void roots(std::vector<std::complex<double>>& results)
	{
		size_t order = coeffs.size() - 1;
		Eigen::VectorXd coeffs_vec
			= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(coeffs.data(), coeffs.size());
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(order, order);
		A.block(1, 0, order - 1, order - 1) = Eigen::MatrixXd::Identity(order - 1, order - 1);
		A.block(0, 0, 1, order) = -coeffs_vec.transpose().block(0, 1, 1, order) / coeffs_vec(0, 0);

		eigensolver.compute(A, false);
		results.resize(A.cols());
		for (int i = 0; i < A.cols(); ++i)
		{
			results[i] = eigensolver.eigenvalues().col(0)[i];
		}
	}

	std::vector<std::complex<double>>
		roots()
	{
		size_t order = coeffs.size() - 1;
		Eigen::VectorXd coeffs_vec
			= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(coeffs.data(), coeffs.size());
		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(order, order);
		A.block(1, 0, order - 1, order - 1) = Eigen::MatrixXd::Identity(order - 1, order - 1);
		A.block(0, 0, 1, order) = -coeffs_vec.transpose().block(0, 1, 1, order) / coeffs_vec(0, 0);

		eigensolver.compute(A, false);
		std::vector<std::complex<double>> results(A.cols());
		results.resize(A.cols());
		for (int i = 0; i < A.cols(); ++i)
		{
			results[i] = eigensolver.eigenvalues().col(0)[i];
		}

		return results;
	}

	Polynomial
		derivative() const
	{
		size_t order = coeffs.size() - 1;
		if (order <= 0)
		{
			// The derivative is alway 0 if the polynomial is a constant value
			return Polynomial({ 0.0 });
		}

		std::vector<double> d_coeffs;
		d_coeffs.resize(order);
		for (int i = 0; i < order; ++i)
		{
			d_coeffs[i] = coeffs[i] * (order - i);
		}
		Polynomial result(d_coeffs);

		return result;
	}

	Polynomial integral() const {
		size_t order = coeffs.size() + 1;

		std::vector<double> i_coeffs;
		i_coeffs.resize(order);
		for (int i = 0; i < order - 1; ++i)
		{
			i_coeffs[i] = coeffs[i] / (order - 1 - i);
		}
		i_coeffs[order - 1] = 0.0; 
		Polynomial result(i_coeffs);

		return result;
	}

	friend std::ostream& operator<<(std::ostream& os, const Polynomial& poly)
	{
		os << "(";
		for (int i = 0; i < poly.coeffs.size(); ++i)
		{
			os << poly.coeffs[i];
			if (i != poly.coeffs.size() - 1) os << ", ";
		}
		os << ")";
		return os;
	}

	friend Polynomial operator+(Polynomial p1, Polynomial p2)
	{
		size_t p1_order = p1.coeffs.size() - 1;
		size_t p2_order = p2.coeffs.size() - 1;
		size_t p_order = std::max(p1_order, p2_order);

		std::vector<double> coeffs(p_order + 1, 0.0);
		for (int i = 0; i <= p_order; ++i)
		{
			if (i <= p1_order) coeffs[p_order - i] += p1.coeffs[p1_order - i];
			if (i <= p2_order) coeffs[p_order - i] += p2.coeffs[p2_order - i];
		}

		Polynomial p(coeffs);
		return p;
	}
};

#endif // POLYNOMIAL_H

