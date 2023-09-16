#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <chrono>
#include <thread>

#include "sensor/sensor.h"
#include <wiringPi.h>

#define delay_ms(a) usleep(a*1000)
using namespace std;
using namespace std::chrono;

IMU mpu;

static auto startTime = std::chrono::system_clock::now();
static auto endTime = std::chrono::system_clock::now();

static void imuCallback(){
	// startTime = std::chrono::system_clock::now();
	mpu.update();
	// std::cout << std::chrono::duration_cast<microseconds>(startTime - endTime).count() << " microseconds\n";
	// endTime = std::chrono::system_clock::now();
}

void test(){
	do{
		printf("Start Measuring");
		mpu.setZero();
		mpu.stopMeasure();
		printf("End Measuring");
		printf("Dist = %2.2f,%2.2f,%2.2f", mpu.measuredPos.distance[0],mpu.measuredPos.distance[1],mpu.measuredPos.distance[2]);
	}while(1);
}

int main() {
	wiringPiSetup();
	mpu.init();
	mpu.getOffset();
	wiringPiISR(0,INT_EDGE_FALLING,imuCallback);

	while(1){
		printf("Yaw = %2.2f, Pitch = %2.2f, Roll = %2.2f \t Accel = %2.2f, %2.2f, %2.2f \t Gravity = %2.2f, %2.2f, %2.2f \t LinAccel = %2.2f, %2.2f, %2.2f \n",
		mpu.currenPos.ypr[0],mpu.currenPos.ypr[1],mpu.currenPos.ypr[2],
		mpu.currenPos.accel[0],mpu.currenPos.accel[1],mpu.currenPos.accel[2],
		mpu.currenPos.gravity.x,mpu.currenPos.gravity.y,mpu.currenPos.gravity.z,
		mpu.currenPos.linAccel[0], mpu.currenPos.linAccel[1],mpu.currenPos.linAccel[2]);
		std::this_thread::sleep_for(5ms);
	}


	return 0;
}
