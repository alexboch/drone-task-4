#ifndef MATH_HPP
#define MATH_HPP

#include <Eigen/Dense>
#include <typesData.hpp>

class Math
{
	public:
		static Eigen::Matrix3d	rotationMatrix(double roll, double pitch, double yaw);
		static Eigen::Matrix3d	rotationMatrix2d(double angle);
		static double			squaring(const double &argument);
		static double			c(const double angl);
		static double			s(const double angl);
		static double 			degreesToRadians(double degrees);
		static VectorXd_t		matrixToVectorXd_t(MatrixXd_t matrix, unsigned int indexRows);
		static double 			limitAngle(double angle);
};

#endif