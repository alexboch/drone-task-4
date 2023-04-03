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
    Eigen::Vector3d enginesVector, gravityVector;
    enginesVector << 0.0, 0.0, enginesForce;
    gravityVector << 0.0, 0.0, -GRAVITY_ACCELERATION;
    //Вектор ускорений поступательного движения по x, y, z
    VectorXd_t accelerationLinear = 1.0 / paramsQuadrotor->mass * rotationMatrix * enginesVector + gravityVector;

    //2.Динамика вращательного движения
    Eigen::Vector3d inertialTensor;
    inertialTensor << paramsQuadrotor -> Ixx, 0.0, 0.0, 
    0.0, paramsQuadrotor -> Iyy, 0.0,
    0.0, 0.0, paramsQuadrotor->Izz;
    auto inverseInertialTensor = inertialTensor.inverse();
    //Момент тяги
    Eigen::Vector3d thrustMoment;
    thrustMoment << paramsQuadrotor->lengthOfFlyerArms * paramsQuadrotor->b * (Math::squaring(rotorsAngularVelocity[0]) - Math::squaring(rotorsAngularVelocity[2])),
    paramsQuadrotor->lengthOfFlyerArms * paramsQuadrotor->b * (Math::squaring(rotorsAngularVelocity[3]) - Math::squaring(rotorsAngularVelocity[2])),
    paramsQuadrotor->d * (Math::squaring(rotorsAngularVelocity[3] + Math::squaring(rotorsAngularVelocity[1]) - 
    Math::squaring(rotorsAngularVelocity[0] - Math::squaring(rotorsAngularVelocity[2]))));
    Eigen::Vector3d angVel;
    angVel << lastStateVector.PitchRate, lastStateVector.RollRate, lastStateVector.YawRate;
    Eigen::Vector3d angularAccelerations;
    angularAcceleration << inverseInertialTensor * (thrustMoment -  angVel * (inertialTensor * angVel));
    //todo: Проинтегрировать

    StateVector nextStateVector();
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
