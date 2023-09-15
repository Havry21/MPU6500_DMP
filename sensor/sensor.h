#pragma once
#include <stdint.h>
#include "../helper_3dmath.h"

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3
constexpr float accelLSB = 1 / 16384.0;
constexpr float G = 9.80665;

struct PositionData
{
    float ypr[3];
    Quaternion q;
    float temp;
    float gyro[3];
    float accel[3];
    float linAccel[3];
    float distance[3];
    VectorFloat gravity; // [x, y, z]            gravity vector

};

class IMU
{
private:
    Quaternion q_offset;
    int dmpRead(PositionData *_pos);
    const int offsetСircles = 50;
public:
    // MPU control/status vars
    uint8_t devStatus; // return status after each device operation
    //(0 = success, !0 = error)
    uint8_t fifoCount;      // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    int16_t a[3]; // [x, y, z]            accel vector
    int16_t g[3]; // [x, y, z]            gyro vector
    int32_t _q[4];
    int32_t t;

    int r;
    int initialized = 0;
    int dmpReady = 0;
    float lastval[3];
    int16_t sensors;

    PositionData currenPos;
    PositionData offsetPos;
    PositionData zeroPos;
    PositionData measuredPos;

    // float ypr[3];
    // Quaternion q;
    // float temp;
    // float gyro[3];
    // float accel[3];
    // float linAccel[3];
    // float distance[3];

    // float gyroOffset[3];
    // float accelOffset[3];

    uint8_t rate = 40;
    bool enableMeasure = false;

    IMU();
    void setZero();
    void stopMeasure();
    void measurePosition();
    int init();
    int update();
    int getOffset();
    int close();
    uint8_t GetGravity(VectorFloat *v, Quaternion *q);
    uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
    uint8_t GetGyro(int32_t *data, const uint8_t *packet);
};
