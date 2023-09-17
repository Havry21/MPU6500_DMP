#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <chrono>
#include <thread>
#include <unistd.h>

#include "sensor/sensor.h"
#include <wiringPi.h>
#include "I2Cdev/I2Cdev.h"
#include "regulator/regulator.h"

#include <mutex>
std::mutex g_lock;

using namespace std;
using namespace std::chrono;

IMU mpu;
regulator motorReg = regulator(&mpu.currenPos.ypr[1], 100.0);

static auto startTime = std::chrono::system_clock::now();
static auto endTime = std::chrono::system_clock::now();

static void imuCallback(){
	// startTime = std::chrono::system_clock::now();
	mpu.update();
	// std::cout << std::chrono::duration_cast<microseconds>(startTime - endTime).count() << " microseconds\n";
	// endTime = std::chrono::system_clock::now();
	motorReg.work();

}




int main() {
	wiringPiSetup();
	mpu.init();
	wiringPiISR(0,INT_EDGE_FALLING,imuCallback);
	// 0 - 17 пин на самом деле

	// motorReg.stop();
	motorReg.start(100,0,mpu.currenPos.ypr[1]);
	while(1){

		// printf("Yaw = %2.2f, Pitch = %2.2f, Roll = %2.2f \t Accel = %2.2f, %2.2f, %2.2f \t Gravity = %2.2f, %2.2f, %2.2f \t LinAccel = %2.2f, %2.2f, %2.2f \n",
		// mpu.currenPos.ypr[0],mpu.currenPos.ypr[1],mpu.currenPos.ypr[2],
		// mpu.currenPos.accel[0],mpu.currenPos.accel[1],mpu.currenPos.accel[2],
		// mpu.currenPos.gravity.x,mpu.currenPos.gravity.y,mpu.currenPos.gravity.z,
		// mpu.currenPos.linAccel[0], mpu.currenPos.linAccel[1],mpu.currenPos.linAccel[2]);
		// std::this_thread::sleep_for(5ms);
	}


	return 0;
}
