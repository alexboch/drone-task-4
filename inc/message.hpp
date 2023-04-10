#ifndef MESSAGES_HPP
#define MESSAGES_HPP


#pragma pack(push,1) // выравнивание в памяти
struct StateVector
{
	// Положение ЛА в стартовой СК
	double X = 0; 
	double Y = 0;
	double Z = 0;
	// Скорость ЛА в стартовой СК
	double VelX = 0;
	double VelY = 0;
	double VelZ = 0;
	// Угловое положение ЛА
	double Pitch = 0;
	double Roll = 0;
	double Yaw = 0;
	// Угловая скорость ЛА
	double PitchRate = 0;
	double RollRate = 0;
	double YawRate = 0;
	// Метка времени симуляции
	double timeStamp = 0;
};
#pragma pack(pop) // выравнивание в памяти


#endif