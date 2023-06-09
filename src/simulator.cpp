#include "simulator.hpp"


Simulator::Simulator(const ParamsQuadrotor &paramsQuadrotor,
				  const ParamsSimulator &paramsSimulator,
				  const ParamsControlSystem &paramsControlSystem)
{	
	// Объект математической модели БЛА
	mathModelQuadrotor = new MathModelQuadrotor(&paramsQuadrotor, &paramsSimulator);
	// Система планирования движения ЛА, осуществляет рассчет полиномов траектории
	// по массиву заданых точек(создается на усмотрение разработчика)
	// В самой простой интерпретации при достижении аппаратом окрестности
	// заданой точки отправляет новое пространственное положение на вход системы управления.
	// В случае создания оптимальной траектории рассчитывает полином положения от времени и
	// минимизирует траекторию согласно заданому критерию(см лекции)

	double yaw = M_PI / 6.0;//Целевое рыскание в 30 градусов
	motionPlanner = new MotionPlanner(0.0);
	// объект системы управления БЛА
	controlSystem = new UAVControlSystem(&paramsControlSystem, &paramsSimulator, &paramsQuadrotor, motionPlanner);

	this->paramsSimulator = paramsSimulator;
	
	
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	// sock = socket(AF_INET, SOCK_DGRAM, 0);
	// Заполняем структуру для адресации при отправке пакетов
	memset((char *) &address, 0, sizeof(address));
	address.sin_family = AF_INET;
	//ставим номер порта
	address.sin_port = htons(PORT);

	slen=sizeof(address);
	//debug
	std::cout << "sim init" << std::endl;
}

Simulator::~Simulator()
{
	delete mathModelQuadrotor;
	delete motionPlanner;
	delete controlSystem;
}




/**
 * @brief основной метод запускающий процесс симуляции
 * 
 */
void Simulator::run(std::vector<Eigen::Vector4d> trajectoryCoords)
{
	std::cout << "run sim" << std::endl;
	int n = trajectoryCoords.size();
	MatrixXd_t pointsRowMatrix(n, 3);
	VectorXd_t timeVector(n);//Вектор-столбец времени
	for(int i = 0; i < n; i++)
	{
		Eigen::Vector3d coordsRow;
		for(int j = 0; j < 3; j++)
		{
			coordsRow(j) = trajectoryCoords[i](j);//Координаты точки в пространстве
		}
		double time = trajectoryCoords[i].w();
		pointsRowMatrix.row(i) = coordsRow;
		timeVector(i) = time;
	}
	std::cout << "fill time vector" << std::endl;

	stateVector = getInitialState();
	motionPlanner->initializeTrajectory(stateVector, pointsRowMatrix, timeVector);
	std::cout<<"trajectory initialized"<<std::endl;
	//motionPlanner->initializeTrajectory()
	// Выполняем моделирование системы в цикле
	for (double t = 0; t < paramsSimulator.simulationTotalTime; t += paramsSimulator.dt)
	{
		TargetPoints_t targetPoint = motionPlanner->getCurrentTargetPoint(stateVector);

		// тут необходимо вызывать методы для получения комманд управления
		VectorXd_t motorsCmd = controlSystem->calculateMotorVelocity(stateVector, targetPoint, t);
		// тут необходимо вызывать методы для вычисления функции правых частей
		// математической модели, выполнять интегрирование приращений и формирование вектора состояния
		// Прим. Вектор состояния предлагается использовать в виде структуры(описание структуры в message.hpp)
		stateVector = this->mathModelQuadrotor->calculateStateVector(stateVector, motorsCmd);
		// устанавливаем метку времени
		stateVector.timeStamp = t;

		if((int)round(t/paramsSimulator.dt) % 25 == 0)
		{
			std::cout<<"X:"<<stateVector.X<<" Y:"<<stateVector.Y<<" Z:"<<stateVector.Z
			<<"Yaw:"<<stateVector.Yaw<<std::endl;
		}
		// Отправляем вектор состояния
		sendMessage(stateVector);
		// Для простейшей имитации движения аппарата в реальном времени 
		// можно вызывать задержку или воспользоваться прерываниями
		usleep(paramsSimulator.dt * 1e6);
	}
}



void Simulator::sendMessage(const StateVector &stateVector)
{
	// Переведем вектор состояния
	if (sendto(sock, &stateVector, sizeof(stateVector) , 0 , (struct sockaddr *) &address, slen)==-1)
	{
		// выводим ошибку в терминал
		perror("sendto()");
		// завершаем выполнение программы
		exit(1);
	}
}

StateVector Simulator::getInitialState()
{
	StateVector sv;
	sv.Z = 1.0;
	sv.Yaw = 0.0;//PI/6 = 30 grad
	return sv;
}

//Проверяем математическую модель
void Simulator::testCalcNextState()
{
	StateVector initialState;
	initialState.X = 2;
	initialState.Y = 5;
	initialState.Z = 1;
}