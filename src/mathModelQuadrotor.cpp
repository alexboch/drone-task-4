#include "mathModelQuadrotor.hpp"

MathModelQuadrotor::MathModelQuadrotor(const ParamsQuadrotor *paramsQuadrotor, const ParamsSimulator *paramsSimulator)
{
    this->paramsQuadrotor = paramsQuadrotor;
    this->paramsSimulator = paramsSimulator;

}

/**
 * @brief метод рассчитывающий вектор состояния системы
 * 
 * @param rotorsAngularVelocity угловая скорость вращения роторов
 * @return вектор состояния системы состоящий из 9 компонент(позиция, ориентация, угловая скорость)
 */
StateVector MathModelQuadrotor::calculateStateVector(StateVector &lastStateVector, VectorXd_t rotorsAngularVelocity)
{
   
    //Проинтегрировать
    MatrixXd_t rightParts = functionRight(lastStateVector, rotorsAngularVelocity);
    auto enginesVector = Math::matrixToVectorXd_t(rightParts, 0);
    auto angAccelerationVector = Math::matrixToVectorXd_t(rightParts, 1);

    this -> angularAcceleration = angAccelerationVector;
    this->angularVelocity += angularAcceleration * this->paramsSimulator->dt;
    this->orientation += angularVelocity * paramsSimulator->dt;//Угловое положение

    double nextRoll = orientation[0];
    double nextPitch = orientation[1];
    double nextYaw = orientation[2];
    auto rotationMatrix = Math::rotationMatrix(nextRoll, nextPitch, nextYaw);//Матрица поворота из углов
    
    Eigen::Vector3d gravityVector;
    gravityVector << 0.0, 0.0, -GRAVITY_ACCELERATION;
    //Вектор ускорений поступательного движения по x, y, z в стартовой системе координат
    Eigen::Vector3d accelerationLinear = 1.0 / paramsQuadrotor->mass * rotationMatrix * enginesVector + gravityVector;

    this->acceleration = accelerationLinear;
    this->velocity << lastStateVector.VelX, lastStateVector.VelY, lastStateVector.VelZ;
    this->velocity += accelerationLinear * this->paramsSimulator->dt;
    this->position << lastStateVector.X, lastStateVector.Y, lastStateVector.Z;
    this->position += velocity * paramsSimulator->dt;//Положение в стартовой системе координат


    StateVector nextStateVector;
    nextStateVector.Pitch = nextPitch;
    nextStateVector.Roll = nextRoll;
    nextStateVector.Yaw = nextYaw;
    nextStateVector.VelX = velocity.x();
    nextStateVector.VelY = velocity.y();
    nextStateVector.VelZ = velocity.z();
    nextStateVector.X = position.x();
    nextStateVector.Y = position.y();
    nextStateVector.Z = position.z();
    nextStateVector.PitchRate = angularVelocity.x();
    nextStateVector.RollRate = angularVelocity.y();
    nextStateVector.YawRate = angularVelocity.z();
    nextStateVector.timeStamp = lastStateVector.timeStamp + paramsSimulator->dt;
    return nextStateVector;
}

/**
 * @brief Метод, содержащий в себе функцию правых частей
 * 
 * @param angularVelocityRotors Вектор угловых скоростей роторов
 * @return Вектор ускорений, вектор угловых ускорений
 */
MatrixXd_t	MathModelQuadrotor::functionRight(StateVector &lastStateVector, VectorXd_t rotorsAngularVelocity)
{
        
    //Eigen::Matrix<double, 2, 3> result;
    MatrixXd_t result(2, 3);
        //2.Динамика вращательного движения
    MatrixXd_t inertialTensor(3, 3);
    inertialTensor << paramsQuadrotor -> Ixx, 0.0, 0.0, 
    0.0, paramsQuadrotor -> Iyy, 0.0,
    0.0, 0.0, paramsQuadrotor->Izz;
    MatrixXd_t inverseInertialTensor = inertialTensor.inverse();
    //Момент тяги
    Eigen::Vector3d thrustMoment;
    thrustMoment << paramsQuadrotor->lengthOfFlyerArms * paramsQuadrotor->b * (Math::squaring(rotorsAngularVelocity[0]) - Math::squaring(rotorsAngularVelocity[2])),
    paramsQuadrotor->lengthOfFlyerArms * paramsQuadrotor->b * (Math::squaring(rotorsAngularVelocity[3]) - Math::squaring(rotorsAngularVelocity[2])),
    paramsQuadrotor->d * (Math::squaring(rotorsAngularVelocity[3] + Math::squaring(rotorsAngularVelocity[1]) - 
    Math::squaring(rotorsAngularVelocity[0] - Math::squaring(rotorsAngularVelocity[2]))));
    Eigen::Vector3d angVel;
    angVel << lastStateVector.PitchRate, lastStateVector.RollRate, lastStateVector.YawRate;
    Eigen::Vector3d angularAccelerations;
    Eigen::Vector3d p1;
    p1 = inertialTensor * angVel;
    angularAcceleration = inverseInertialTensor * (thrustMoment -  angVel.cross(p1));
    
    double enginesForce = 0.0;//Общая сила, создаваемая всеми двигателями
    
    for(int i = 0; i < this->paramsQuadrotor->numberOfRotors; i++)
    {
        enginesForce += Math::squaring(rotorsAngularVelocity[i]);
    }
    enginesForce *= paramsQuadrotor->b;
    Eigen::Vector3d enginesVector;
    enginesVector << 0.0, 0.0, enginesForce;

    result.row(0) = enginesVector;
    result.row(1) = angularAcceleration;
    return result;
}
