#include <PIDController.hpp>


PIDController::PIDController(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}


void PIDController::SetTargetValue(double targetValue)
{
    _targetValue = targetValue;
}

double PIDController::SetCurrentValue(double currentValue)
{
    double err = _targetValue - currentValue;
    _errorSum += err;
    double cmd = _kp * err + _ki * _errorSum + _kd * (err - _prevError);
    _prevError = err;
    return cmd;
}
