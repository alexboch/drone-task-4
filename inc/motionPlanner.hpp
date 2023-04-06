#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <typesData.hpp>
#include <math.hpp>
#include <message.hpp>

class MotionPlanner
{
	public:
		void			calculateTrajectory(StateVector stateVector, MatrixXd_t targetPoints, VectorXd_t timeTrajectory);
		VectorXd_t		getRowsCoeffX(unsigned int indexRows);
		VectorXd_t		getRowsCoeffY(unsigned int indexRows);
		VectorXd_t		getRowsCoeffZ(unsigned int indexRows);
		unsigned int	getSizeTimeTrajectory();
		double			getTimeTrajectory(unsigned int indexPoint);
		double			calculateDesiredPosition(VectorXd_t c, double t);
		double			calculateDesiredVelocity(VectorXd_t c, double t);
		double			calculateDesiredAcceleration(VectorXd_t c, double t);
		MotionPlanner();
	private:

		
		// Массив коэффициентов для каждой точки(строка - коэффициенты для точки)
		MatrixXd_t	xCoeff;
		MatrixXd_t	yCoeff;
		MatrixXd_t	zCoeff;
		// промежутки времени за которое БЛА пролетает траекторию
		VectorXd_t	timeTrajectory;

		MatrixXd_t		trajectoryGenerator(VectorXd_t currentPoints, VectorXd_t targetPoints, double T);
		//Получить вектор множителей коэффициентов для полинома положения 
		Vectord_t getMultsPositionPolynom(double time);
		//Получить вектор множителей коэффициентов для полинома скорости 
		Vectord_t getMultsVelocityPolynom(double time);
		//Получить вектор множителей коэффициентов для полинома ускорения 
		Vectord_t getMultsAccelerationPolynom(double time);


		Matrixd_t getTMatrix(double time1, double time2);

		//Получить вектор-столбец с коэффицентами для заданных начальных и конечных условий
		ColVectord_c getCoeffVector(ColVectord_c conditions, double time1, double time2);

		//Eigen::Matrix<double, 6, 6> getTMatrix(Eigen::Vector)
};

#endif