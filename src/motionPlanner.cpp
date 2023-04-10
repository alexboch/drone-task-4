#include "motionPlanner.hpp"


MotionPlanner::MotionPlanner(double yaw)
{
	_yaw = yaw;
}

/**
 * @brief функция для расчёта траектории(заполняет коэффициенты полинома)
 * 
 * @param stateVector вектор состояния
 * @param targetPoints матрица координат точек (nx3), где n - кол-во точек
 * @param timeTrajectory массив времени длины n, за которое требуется пролететь от точки до точки
 */
void			MotionPlanner::initializeTrajectory(StateVector stateVector, MatrixXd_t targetPoints, VectorXd_t timeTrajectory)
{
	
	this->targetPointsRowMatrix = targetPoints;
	this->timeTrajectory = timeTrajectory;
	// auto targetCoords = targetPoints.row(0);

}


VectorXd_t		MotionPlanner::getRowsCoeffX(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(xCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffY(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(yCoeff, indexRows));
}

VectorXd_t		MotionPlanner::getRowsCoeffZ(unsigned int indexRows)
{
	return (Math::matrixToVectorXd_t(zCoeff, indexRows));
}

double			MotionPlanner::getTimeTrajectory(unsigned int indexPoint)
{
	return (timeTrajectory(indexPoint));
}

unsigned int	MotionPlanner::getSizeTimeTrajectory()
{
	return (timeTrajectory.size());
}

/**
 * @brief функция для расчёта минимума по рывку
 * 
 * @param c 
 * @param t 
 * @return double 
 */
double			MotionPlanner::calculateDesiredPosition(VectorXd_t c, double t)
{
}

double			MotionPlanner::calculateDesiredVelocity(VectorXd_t c, double t)
{
}

double			MotionPlanner::calculateDesiredAcceleration(VectorXd_t c, double t)
{
}

MatrixXd_t		MotionPlanner::trajectoryGenerator(VectorXd_t currentPoints, VectorXd_t targetPoints, double T)
{
	
}

// Получить вектор множителей коэффициентов для полинома положения
Vectord_t MotionPlanner::getMultsPositionPolynom(double time)
{
	Vectord_t mults;
	
	for(int p = 5; p >=0; p--)
	{
		mults[5 - p]=pow(time, p); 
	}
	return mults;
}

// Получить вектор множителей коэффициентов для полинома скорости
Vectord_t MotionPlanner::getMultsVelocityPolynom(double time)
{
	Vectord_t mults;
	int m = 5;
	int p = 4;
	for(int i = 0; i < 6; i++)
	{
		
		mults[i] = m * pow(time, p);
		m--;
		p--;
	}
	return mults;
}

// Получить вектор множителей коэффициентов для полинома ускорения
Vectord_t MotionPlanner::getMultsAccelerationPolynom(double time)
{
	Vectord_t mults;
	mults << 20 * pow(time, 3), 12 * time * time, 6 * time, 2, 0, 0;
	return mults;
}

// Получить вектор-столбец с коэффицентами для заданных начальных и конечных условий
ColVectord_c MotionPlanner::getCoeffVector(ColVectord_c conditions, double time1, double time2)
{
	auto tMatrix = this->getTMatrix(time1, time2);
	ColVectord_c coeffColVector = tMatrix.inverse() * conditions;
	return coeffColVector;
}

Matrixd_t MotionPlanner::getTMatrix(double time1, double time2)
{
	Matrixd_t tMatrix;
	tMatrix.row(0) = getMultsPositionPolynom(time1);
	tMatrix.row(1) = getMultsPositionPolynom(time2);
	tMatrix.row(2) = getMultsVelocityPolynom(time1);
	tMatrix.row(3) = getMultsVelocityPolynom(time2);
	tMatrix.row(4) = getMultsAccelerationPolynom(time1);
	tMatrix.row(5) = getMultsAccelerationPolynom(time2);
	return tMatrix;
}

bool MotionPlanner::checkRadius(StateVector state, Eigen::Vector3d point)
{
	const double eps = 0.1;
	return sqrt(Math::squaring(state.X - point.x()) + Math::squaring(state.Y - point.y()) + Math::squaring(state.Z - point.z())) < eps;
}

TargetPoints_t MotionPlanner::getCurrentTargetPoint(StateVector stateVector)
{
	auto currTargetPoint = targetPointsRowMatrix.row(_currentPointIndex);
	//Если находимся в радиусе текущей целевой точки, то переходим к следующей
	if(checkRadius(stateVector, currTargetPoint))
	{
		if(_currentPointIndex < _countPoints - 1) 
		{
			_currentPointIndex++;
			//Нужно пересчитать коэффициенты полинома
			
		}
	}
	TargetPoints_t resultPoint;

	auto currentCoords = targetPointsRowMatrix.row(_currentPointIndex);
	resultPoint << currentCoords.x(), currentCoords.y(), currentCoords.z();
	resultPoint << _yaw;
	return resultPoint;
}
