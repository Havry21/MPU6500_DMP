#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <chrono>
#include "sensor/sensor.h"
#include <thread>
#include <mutex>


#define delay_ms(a) usleep(a*1000)
using namespace std;
using namespace std::chrono;
IMU mpu;
std::mutex mut;

void getMpuData(){
	while (1)
	{
		mut.lock();
		mpu.update();
		mut.unlock();
		std::this_thread::sleep_for(std::chrono::microseconds(5));
	}
}
void test(){
	do{
		std::this_thread::sleep_for(std::chrono::seconds(10));
		printf("Start Measuring");
		mpu.setZero();
		std::this_thread::sleep_for(std::chrono::seconds(5));
		mpu.stopMeasure();
		printf("End Measuring");
		printf("Dist = %2.2f,%2.2f,%2.2f", mpu.measuredPos.distance[0],mpu.measuredPos.distance[1],mpu.measuredPos.distance[2]);
	}while(1);
}

int main() {
	mpu.init();
	mpu.getOffset();
	std::thread t1(&getMpuData);
	std::thread t2(&test);

	t1.join();
	t2.join();



	return 0;
}
