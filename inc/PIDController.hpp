#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP


#include <typesData.hpp>
class PIDController{

private:
    double _prevError = 0.0;
    double _errorSum = 0.0;
    double _targetValue=0.0;
    double _currentValue=0.0;
    double _kp, _ki, _kd;
    double _minValue, _maxValue;
    double saturation(double arg);

public:
    void SetTargetValue(double targetValue);

    /// @brief Вычисляет ошибку и управляющее воздействие
    /// @param currentValue текущее значение
    /// @return Управляющее воздействие  на основе вычисленной ошибки
    double SetCurrentValue(double currentValue);

    PIDController(double kp, double ki, double kd, double maxValue = INFINITY, double minValue = -INFINITY);
    PIDController();

};


#endif