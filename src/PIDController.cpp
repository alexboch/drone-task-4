#include <PIDController.hpp>

PIDController::PIDController()
{
    
}

PIDController::PIDController(double kp, double ki, double kd, double maxValue, double minValue)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _maxValue = maxValue;
    _minValue = minValue;
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
    return saturation(cmd);
}

double PIDController::saturation(double arg)
{
    if(arg < _minValue)
    {
        return _minValue;
    }
    if(arg > _maxValue)
    {
        return _maxValue;
    }
    return arg;
}
