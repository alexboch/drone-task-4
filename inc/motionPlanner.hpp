#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <typesData.hpp>
#include <math.hpp>
#include <message.hpp>

class MotionPlanner
{
	public:
		void			initializeTrajectory(StateVector stateVector, MatrixXd_t targetPoints, VectorXd_t timeTrajectory);
		

		VectorXd_t		getRowsCoeffX(unsigned int indexRows);
		VectorXd_t		getRowsCoeffY(unsigned int indexRows);
		VectorXd_t		getRowsCoeffZ(unsigned int indexRows);
		//Получить текущую целевую точку в виде вектора (x, y, z, рыскание)
		TargetPoints_t 	getCurrentTargetPoint(StateVector stateVector);

		unsigned int	getSizeTimeTrajectory();
		double			getTimeTrajectory(unsigned int indexPoint);
		double			calculateDesiredPosition(VectorXd_t c, double t);
		double			calculateDesiredVelocity(VectorXd_t c, double t);
		double			calculateDesiredAcceleration(VectorXd_t c, double t);
		


		/// @brief 
		/// @param yaw угол рыскания, будет константным
		MotionPlanner(double yaw);


	private:


		int _countPoints;
		int _currentPointIndex = 0;//Индекс текущей целевой точки

		double _yaw;
		// Массив коэффициентов для каждой точки(строка - коэффициенты для точки)
		MatrixXd_t	xCoeff;
		MatrixXd_t	yCoeff;
		MatrixXd_t	zCoeff;
		// промежутки времени за которое БЛА пролетает траекторию
		VectorXd_t	timeTrajectory;
		//Матрица целевых координат
		Eigen::Matrix<double, -1, 3> targetPointsRowMatrix;

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

		ColVectord_c getConditionsVector();
		//Eigen::Matrix<double, 6, 6> getTMatrix(Eigen::Vector)

		bool checkRadius(StateVector state, Eigen::Vector3d targetPoint);
};

#endif