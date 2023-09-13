#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "sensor/sensor.h"

#define delay_ms(a) usleep(a*1000)

IMU mpu;

int main() {
	mpu.init();

	do{
		mpu.update();
		printf("yaw = %2.1f\tpitch = %2.1f\troll = %2.1f \t_accel = %2.1f, %2.1f, %2.1f\n",
		 mpu.ypr[YAW], mpu.ypr[PITCH],
		 mpu.ypr[ROLL],mpu.accel[0],mpu.accel[1],mpu.accel[2]);
		delay_ms(5);
	}while(1);

	return 0;
}
