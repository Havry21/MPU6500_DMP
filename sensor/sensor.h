#pragma once
#include <stdint.h>
#include "../helper_3dmath.h"

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3

class IMU{
public:
        // MPU control/status vars
    uint8_t devStatus;      // return status after each device operation
    //(0 = success, !0 = error)
    uint8_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    int16_t a[3];              // [x, y, z]            accel vector
    int16_t g[3];              // [x, y, z]            gyro vector
    int32_t _q[4];
    int32_t t;

    VectorFloat gravity;    // [x, y, z]            gravity vector

    int r;
    int initialized = 0;
    int dmpReady = 0;
    float lastval[3];
    int16_t sensors;

    float ypr[3];
    Quaternion q; 
    float temp;
    float gyro[3];
    float accel[3];

    float gyroOffset[3];
    float accelOffset[3];
    
    float compass[3];

    uint8_t rate = 40;

    IMU();
    int init();
    int update();
    int getOffset();
    int close();
    uint8_t GetGravity(VectorFloat *v, Quaternion *q);
    uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity); 
    uint8_t GetGyro(int32_t *data, const uint8_t* packet);
};
