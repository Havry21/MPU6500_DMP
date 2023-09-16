#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "../helper_3dmath.h"
#include "../inv_mpu_lib/inv_mpu.h"
#include "../inv_mpu_lib/inv_mpu_dmp_motion_driver.h"
#include "sensor.h"

#define wrap_180(x) (x < -180 ? x + 360 : (x > 180 ? x - 360 : x))
#define delay_ms(a) usleep(a * 1000)




IMU::IMU()
{
}

int IMU::init()
{
	dmpReady = 1;
	initialized = 0;
	for (int i = 0; i < DIM; i++)
	{
		lastval[i] = 10;
	}

	// initialize device
	printf("Initializing MPU...\n");
	if (mpu_init(NULL) != 0)
	{
		printf("MPU init failed!\n");
		return -1;
	}
	printf("Setting MPU sensors...\n");
	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0)
	{
		printf("Failed to set sensors!\n");
		return -1;
	}
	printf("Setting GYRO sensitivity...\n");
	if (mpu_set_gyro_fsr(2000) != 0)
	{
		printf("Failed to set gyro sensitivity!\n");
		return -1;
	}
	printf("Setting ACCEL sensitivity...\n");
	if (mpu_set_accel_fsr(2) != 0)
	{
		printf("Failed to set accel sensitivity!\n");
		return -1;
	}

	if (mpu_set_lpf(5) != 0)
	{
		printf("Failed to set filter!\n");
		return -1;
	}
	// verify connection
	printf("Powering up MPU...\n");
	mpu_get_power_state(&devStatus);
	printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed %u\n", devStatus);

	// fifo config
	printf("Setting MPU fifo...\n");
	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0)
	{
		printf("Failed to initialize MPU fifo!\n");
		return -1;
	}

	// load and configure the DMP
	printf("Loading DMP firmware...\n");
	if (dmp_load_motion_driver_firmware() != 0)
	{
		printf("Failed to enable DMP!\n");
		return -1;
	}

	printf("Activating DMP...\n");
	if (mpu_set_dmp_state(1) != 0)
	{
		printf("Failed to enable DMP!\n");
		return -1;
	}

	// dmp_set_orientation()
	// if (dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
	printf("Configuring DMP...\n");
	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL) != 0)
	{
		printf("Failed to enable DMP features!\n");
		return -1;
	}

	printf("Setting DMP fifo rate...\n");
	if (dmp_set_fifo_rate(rate) != 0)
	{
		printf("Failed to set dmp fifo rate!\n");
		return -1;
	}
	printf("Resetting fifo queue...\n");
	if (mpu_reset_fifo() != 0)
	{
		printf("Failed to reset fifo!\n");
		return -1;
	}

	// printf("Set interrupt mode...\n");
	// if (dmp_set_interrupt_mode(DMP_INT_CONTINUOUS) != 0)
	// {
	// 	printf("Failed to set interrupt mode!\n");
	// 	return -1;
	// }
	

	printf("Checking... ");
	do
	{
		delay_ms(1000 / rate); // dmp will habve 4 (5-1) packets based on the fifo_rate
		r = dmp_read_fifo(g, a, _q, &sensors, &fifoCount);
	} while (r != 0 || fifoCount < 5); // packtets!!!
	printf("Done.\n");

	initialized = 1;
	return 0;
}

int IMU::dmpRead(PositionData *_pos)
{
	if (!dmpReady)
	{
		printf("Error: DMP not ready!!\n");
		return -1;
	}
	while (dmp_read_fifo(g, a, _q, &sensors, &fifoCount) != 0);

	_pos->q = _q;

	GetGravity(&_pos->gravity, &_pos->q);
	_pos->gravity.normalize();

	GetYawPitchRoll(_pos->ypr, &_pos->q, &_pos->gravity);

	for (int i = 0; i < DIM; i++)
	{
		_pos->ypr[i] *= 180 / M_PI;
	}

	// unwrap yaw when it reaches 180
	_pos->ypr[0] = wrap_180(_pos->ypr[0]);

	// change sign of Pitch, MPU is attached upside down
	_pos->ypr[1] *= -1.0;

	for (int i = 0; i < DIM; i++)
	{
		// gyro[i]   = (float)(g[DIM-i-1])/131.0/360.0 - gyroOffset[i];
		_pos->accel[i] = ((float)a[i] * accelLSB);
		// if (abs(accel[i]) < dummyThreshold) accel[i] = 0;
	}

	_pos->linAccel[0] = (_pos->accel[0] - _pos->gravity.x) * G;
	_pos->linAccel[1] = (_pos->accel[1] - _pos->gravity.y) * G;
	_pos->linAccel[2] = (_pos->accel[2] - _pos->gravity.z) * G;
	for (int i = 0; i < DIM; i++)
	{
		_pos->distance[i] += _pos->linAccel[i] * 0.25;
	}
	return 0;
}

int IMU::getOffset()
{
	PositionData _pos[offsetСircles];
	int rc;

	for (int i = 0; i < offsetСircles; i++)
	{
		rc = dmpRead(&_pos[i]);
		if (rc < 0)
		{
			return rc;
		}
		usleep(5000);
	}
	return 0;
}

int IMU::update()
{
	dmpRead(&currenPos);
	// if(enableMeasure){
	// 	measurePosition();
	// }
	return 0;
}

int IMU::close()
{
	return 0;
}

void::IMU::stopMeasure(){
	enableMeasure = false;
}

void IMU::setZero(){
	zeroPos = currenPos;
	enableMeasure = true;
}

void IMU::measurePosition(){
	for(int i = 0; i < DIM; i++){
		measuredPos.accel[i] = currenPos.accel[i] - zeroPos.accel[i];
		measuredPos.gyro[i] = currenPos.gyro[i] - zeroPos.gyro[i];
		measuredPos.gyro[i] = currenPos.gyro[i] - zeroPos.gyro[i];
		measuredPos.linAccel[i] = currenPos.linAccel[i] - zeroPos.linAccel[i];
		measuredPos.distance[i] = currenPos.distance[i] - zeroPos.distance[i];
	}
}


uint8_t IMU::GetGravity(VectorFloat *v, Quaternion *q)
{
	v->x = 2 * (q->x * q->z - q->w * q->y);
	v->y = 2 * (q->w * q->x + q->y * q->z);
	v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
	return 0;
}

uint8_t IMU::GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
{
	// yaw: (about Z axis)
	data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(gravity->x / sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(gravity->y / sqrt(gravity->x * gravity->x + gravity->z * gravity->z));
	return 0;
}
