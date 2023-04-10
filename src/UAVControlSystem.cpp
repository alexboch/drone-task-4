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
 * @param stateVector вектор состояния БЛА, приходит из модели динамики
 * @param targetPoints целевая точка(x, y, z, угол рыскания)
 * @param time 
 * @return VectorXd_t 4 скорости для каждого ротора для модели динамики
 */
VectorXd_t	UAVControlSystem::calculateMotorVelocity(StateVector stateVector, TargetPoints_t targetPoints, double time)
{
	auto targetPoint = this->motionPlanner->getCurrentTargetPoint(stateVector);
	double yawDes = targetPoint(3);//Целевое рыскание

	thrustController.SetTargetValue(targetPoint.z());
	double pDes = thrustController.SetCurrentValue(stateVector.Z);//Команда по тяге
	//auto rotationMatrix = Math::rotationMatrix2d(stateVector.Yaw);//todo: Матрица поворота

	Eigen::Matrix2d rotationMatrix;
	pitchController.SetTargetValue(targetPoint.y());
	double pitchDes_st = this->pitchController.SetCurrentValue(stateVector.Y);//тангаж
	rollController.SetTargetValue(targetPoint.x());
	double rollDes_st = this->rollController.SetCurrentValue(stateVector.X);
	Eigen::Vector2d angles;
	angles << pitchDes_st, rollDes_st;
	//Домножим матрицу поворота на упр. возд-я по тангажу и крену, чтобы перепроектировать их в связанную систему с учетом рысканья
	Eigen::Vector2d angles_sv = rotationMatrix * angles;
	double pitchDes = angles_sv(0);
	double rollDes = angles_sv(1);
	this->wxController.SetTargetValue(pitchDes);
	double wX = this->wxController.SetCurrentValue(stateVector.Pitch);
	this->wyController.SetTargetValue(rollDes);
	double wY = wyController.SetCurrentValue(stateVector.Roll);
	this->wzController.SetTargetValue(yawDes);
	double wZ = wzController.SetCurrentValue(stateVector.Yaw);

	//Получим команды по угловым ускорениям
	awxController.SetTargetValue(wX);
	double awx = awxController.SetCurrentValue(stateVector.PitchRate);
	awyController.SetTargetValue(wY);
	double awy = awyController.SetCurrentValue(stateVector.RollRate);
	awzController.SetTargetValue(wZ);
	double awz = awzController.SetCurrentValue(stateVector.YawRate);

	auto rotorsCmd = mixer(pDes, awx, awy, awz);//Смешиваем команды и получаем команды для роторов
	return rotorsCmd;
}

/**
 * @brief Алгоритм смешивания комманд
 * 
 * @return угловая скорость вращения роторов 
 */
Eigen::Vector4d	UAVControlSystem::mixer(double pDes, double wDezX, double wDezY, double wDezZ)
{
	Eigen::Vector4d rotorCommands;
	rotorCommands << pDes + wDezX - wDezZ,
	pDes - wDezY + wDezZ, pDes - wDezX - wDezZ, pDes + wDezY + wDezZ;
	return rotorCommands;
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