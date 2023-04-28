#include 	<simulator.hpp>
#include 	<typesData.hpp>
#include 	<configLoader.hpp>
#include<vector>


void TestRotationMatrix()
{
	Eigen::Vector3d x;
	x<<1, 0, 0;
	double roll = 0, pitch = Math::degreesToRadians(90), yaw = 0;
	auto r = Math::rotationMatrix(roll, pitch, yaw);
	auto x_res = r * x;
	std::cout<<x_res<<std::endl;
}

int	main()
{
	TestRotationMatrix();


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
	Eigen::Vector4d targetPoints[1];
	targetPoints[0]<<1,0,1,0;
	// targetPoints[1]<<0,1,3,20;
	// targetPoints[2]<<1,2,3,30;
	for(int i = 0; i < targetPoints->size(); i++)
		trajectoryCoordinates.push_back(targetPoints[i]);
	// Запускаем симуляцию
	uavSim.run(trajectoryCoordinates);


	return (0);
}