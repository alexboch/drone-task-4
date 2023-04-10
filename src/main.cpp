#include 	<simulator.hpp>
#include 	<typesData.hpp>
#include 	<configLoader.hpp>
#include<vector>

int	main()
{
	// создаем экземпляры структур под параметры симулятора
	ParamsQuadrotor		parmsQuadrotor;
	ParamsSimulator		paramsSimulator;
	ParamsControlSystem	paramsControlSystem;



	// заполнение параметров квадрокоптера и симулятора
	loadModelConfig(pathQuadModelConfig, parmsQuadrotor, paramsSimulator);

	// заполнение параметров
	loadControlSysConfig(pathQuadControlSystemConfig, paramsControlSystem);

	// Создаем объект симулятора
	Simulator	uavSim(parmsQuadrotor, paramsSimulator, paramsControlSystem);
	std::vector<Eigen::Vector4d> trajectoryCoordinates;
	Eigen::Vector4d targetPoints[3];
	targetPoints[0]<<0,0,2,10;
	targetPoints[1]<<0,1,3,20;
	targetPoints[2]<<1,2,3,30;
	for(int i = 0; i < 3; i++)
		trajectoryCoordinates.push_back(targetPoints[i]);
	// Запускаем симуляцию
	uavSim.run(trajectoryCoordinates);


	return (0);
}