#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP


#include <typesData.hpp>
class PIDController{

private:
    double _prevError = 0.0;
    double _errorSum = 0.0;
    double _targetValue;
    double _currentValue;
    double _kp, _ki, _kd;
public:
    void SetTargetValue(double targetValue);


    /// @brief Вычисляет ошибку и управляющее воздействие
    /// @param currentValue текущее значение
    /// @return Управляющее воздействие  на основе вычисленной ошибки
    double SetCurrentValue(double currentValue);

    PIDController(double kp, double ki, double kd);
};


#endif