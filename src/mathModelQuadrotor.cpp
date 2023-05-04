#include "mathModelQuadrotor.hpp"
#include <iostream>

MathModelQuadrotor::MathModelQuadrotor(const ParamsQuadrotor *paramsQuadrotor, const ParamsSimulator *paramsSimulator)
{
    this->paramsQuadrotor = paramsQuadrotor;
    this->paramsSimulator = paramsSimulator;
    this->angularVelocity << 0.0, 0.0, 0.0;
    this->velocity << 0.0, 0.0, 0.0;
    this->orientation << 0.0, 0.0, 0.0;
    this->acceleration << 0.0, 0.0, 0.0;
    this->position << 0.0, 0.0, 0.0;
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
    Eigen::Vector3d accelerationLinear = 1.0 / paramsQuadrotor->mass * rotationMatrix.transpose() * enginesVector + gravityVector;

    this->acceleration = accelerationLinear;
    this->velocity << lastStateVector.VelX, lastStateVector.VelY, lastStateVector.VelZ;
    this->velocity += accelerationLinear * this->paramsSimulator->dt;
    this->position << lastStateVector.X, lastStateVector.Y, lastStateVector.Z;
    this->position += velocity * paramsSimulator->dt;//Положение в стартовой системе координат


    //Корректировка значений в [0, 2pi]
    nextPitch = Math::limitAngle(nextPitch);
    nextRoll = Math::limitAngle(nextRoll);
    nextYaw = Math::limitAngle(nextYaw);
    for(int i = 0; i < 3; i++)
        angularVelocity[i] = Math::limitAngle(angularVelocity[i]);

    StateVector nextStateVector;
    nextStateVector.Pitch = nextPitch;
    nextStateVector.Roll = nextRoll;
    nextStateVector.Yaw = 0.0;
    nextStateVector.VelX = velocity.x();
    nextStateVector.VelY = velocity.y();
    nextStateVector.VelZ = velocity.z();
    nextStateVector.X = position.x();
    nextStateVector.Y = position.y();
    nextStateVector.Y = 0.0;
    nextStateVector.Z = position.z();
    nextStateVector.Z = 1.0;
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
        
    MatrixXd_t result(2, 3);
        //2.Динамика вращательного движения
    MatrixXd_t inertialTensor(3, 3);
    inertialTensor << paramsQuadrotor -> Ixx, 0.0, 0.0, 
    0.0, paramsQuadrotor -> Iyy, 0.0,
    0.0, 0.0, paramsQuadrotor->Izz;
    MatrixXd_t inverseInertialTensor = inertialTensor.inverse();
    //Момент тяги
    Eigen::Vector3d thrustMoment;
    double l = paramsQuadrotor->lengthOfFlyerArms;
    double w0 = rotorsAngularVelocity[0], w1 = rotorsAngularVelocity[1], w2 = rotorsAngularVelocity[2], w3 = rotorsAngularVelocity[3];
    double b = paramsQuadrotor->b, d = paramsQuadrotor->d;
    thrustMoment << l * b * (w0 * w0 - w2 * w2),
                    l* b * (w3 * w3 - w1 * w1),
                    d * (w3 * w3 + w1 * w1 - w0 * w0 - w2* w2);
    //std::cout<<std::endl<<"Thrust moment:"<<thrustMoment<<std::endl;
    Eigen::Vector3d angVel;
    angVel << lastStateVector.PitchRate, lastStateVector.RollRate, lastStateVector.YawRate;
    Eigen::Vector3d wAcc;
    Eigen::Vector3d p1;
    p1 = inertialTensor * angVel;


    auto cp = angVel.cross(p1);
    //std::cout<<cp<<std::endl;
    wAcc = inverseInertialTensor * (thrustMoment -  angVel.cross(p1));
    //std::cout<<std::endl<<"Angular accelerations:"<<wAcc<<std::endl;
    //Константные угловые ускорения для теста
    // wAcc(0) = 0;
    // wAcc(2) = 0;
    //


    double enginesForce = 0.0;//Общая сила, создаваемая всеми двигателями
    
    for(int i = 0; i < this->paramsQuadrotor->numberOfRotors; i++)
    {
        enginesForce += Math::squaring(rotorsAngularVelocity[i]);
    }
    enginesForce *= paramsQuadrotor->b;
    Eigen::Vector3d enginesVector;
    enginesVector << 0.0, 0.0, enginesForce;

    result.row(0) = enginesVector;
    result.row(1) = wAcc;
    return result;
}
