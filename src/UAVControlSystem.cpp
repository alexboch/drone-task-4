#include "UAVControlSystem.hpp"

#define DEBUG
#ifdef DEBUG
	#include <iostream>
#endif

UAVControlSystem::UAVControlSystem(const ParamsControlSystem *paramsControlSystem, const ParamsSimulator *paramsSimulator,
								   const ParamsQuadrotor *paramsQuadrotor, MotionPlanner* pathPlaner)
{
	
	this->thrustController = PIDController(paramsControlSystem->KpPosition(2), paramsControlSystem->KiPosition(2), 
	paramsControlSystem->KpPosition(2), paramsControlSystem->maxThrust);//регулировка тяги по высоте
	//Угловое положение по положению
	this->rollController = PIDController(paramsControlSystem->KpPosition(0), paramsControlSystem->KiPosition(0), 
	paramsControlSystem->KdPosition(0));//крен по x
	this->pitchController = PIDController(paramsControlSystem->KpPosition(1), paramsControlSystem->KiPosition(1),
	paramsControlSystem->KdPosition(1));//Тангаж по y
	
	//Угловая скорость по угловому положению
	this->wxController = PIDController(paramsControlSystem->KpAngle(0), paramsControlSystem->KiAngle(0), 
	paramsControlSystem->KdAngle(0));//По тангажу
	this->wyController = PIDController(paramsControlSystem->KpAngle(1), paramsControlSystem->KiAngle(1), 
	paramsControlSystem->KdAngle(1));//По крену
	this->wzController = PIDController(paramsControlSystem->KpAngle(2), paramsControlSystem->KiAngle(2), 
	paramsControlSystem->KdAngle(2));//По рысканию

	//Угловое ускорение по угловой скорости
	this->awxController = PIDController(paramsControlSystem->KpAngularRate(0), paramsControlSystem->KiAngularRate(0), 
	paramsControlSystem->KdAngularRate(0));
	this->awyController = PIDController(paramsControlSystem->KpAngularRate(1), paramsControlSystem->KiAngularRate(1), 
	paramsControlSystem->KiAngularRate(1));
	this->awzController = PIDController(paramsControlSystem->KpAngularRate(2), paramsControlSystem->KiAngularRate(2), 
	paramsControlSystem->KiAngularRate(2));
	
}

/**
 * @brief расчёт угловых скоростей роторов
 * 
 * @param stateVector вектор состояния БЛА
 * @param targetPoints целевая точка(x, y, z, угол рыскания)
 * @param time 
 * @return VectorXd_t 4 скорости для каждого ротора для модели динамики
 */
VectorXd_t	UAVControlSystem::calculateMotorVelocity(StateVector stateVector, TargetPoints_t targetPoints, double time)
{
	auto targetPoint = this->motionPlanner->getCurrentTargetPoint();

}

/**
 * @brief Алгоритм смешивания комманд
 * 
 * @return угловая скорость вращения роторов 
 */
VectorXd_t	UAVControlSystem::mixer()
{
}

/**
 * @brief функция, которая заполняет вектор желаемой позиции
 * 
 * @param targetPoints массив точек
 */
void		UAVControlSystem::fillDesiredPosition(MatrixXd_t targetPoints)
{
}



/**
 * @brief проверка попадания в радиус целевой позиции
 * 
 * @param targetPoints целевая позиция
 * @return true - принадлежим сфере 
 * @return false - не принадлежим сфере
 */
bool		UAVControlSystem::checkRadius(VectorXd_t targetPoints)
{
	
}


/**
 * @brief перевод из команды по тяги в угловую скорость
 * 
 * @param commandThrust команда по тяги
 * @return угловая скорость ротора
 */
double		UAVControlSystem::commandThrustToOmegaRotors(double commandThrust)
{
}

void 		UAVControlSystem::fillErrors()
{

}

void 		UAVControlSystem::setCurrentState(StateVector currentState)
{
	
}