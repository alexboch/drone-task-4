#include "mathModelQuadrotor.hpp"

MathModelQuadrotor::MathModelQuadrotor(const ParamsQuadrotor *paramsQuadrotor, const ParamsSimulator *paramsSimulator)
{
}

/**
 * @brief метод рассчитывающий вектор состояния системы
 * 
 * @param rotorsAngularVelocity угловая скорость вращения роторов
 * @return вектор состояния системы состоящий из 9 компонент(позиция, ориентация, угловая скорость)
 */
StateVector MathModelQuadrotor::calculateStateVector(StateVector &lastStateVector, VectorXd_t rotorsAngularVelocity)
{
    auto rotationMatrix = Math::rotationMatrix(lastStateVector.Roll, lastStateVector.Pitch, lastStateVector.Yaw);//Матрица поворота из углов
    double enginesForce = 0.0;//Общая сила, создаваемая всеми двигателями
    for(int i = 0; i < this->paramsQuadrotor->numberOfRotors; i++)
    {
        enginesForce += Math::squaring(rotorsAngularVelocity[i]);
    }
    enginesForce *= paramsQuadrotor->b;
    VectorXd_t enginesVector(3), gravityVector(3);
    enginesVector << 0.0, 0.0, enginesForce;
    gravityVector << 0.0, 0.0, -GRAVITY_ACCELERATION;
    //Вектор ускорений поступательного движения по x, y, z
    VectorXd_t accelerationLinear = 1.0 / paramsQuadrotor->mass * rotationMatrix * enginesVector + gravityVector;

}

/**
 * @brief Метод, содержащий в себе функцию правых частей
 * 
 * @param angularVelocityRotors Вектор угловых скоростей роторов
 * @return Вектор ускорений, вектор угловых ускорений
 */
StateVector	MathModelQuadrotor::functionRight(StateVector &lastStateVector, VectorXd_t rotorsAngularVelocity)
{
}
