#pragma once
#include <stdint.h>
#include <chrono>

#define ARDUINO_ADR 0x18

class PIDImpl
{
public:
    PIDImpl(double dt, int8_t max, int8_t min, double Kp, double Kd, double Ki);
    int16_t calculate(double setpoint, double pv);
    double _dt;

private:
    int8_t _max;
    int8_t _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

class regulator
{
private:
    int rate = 10;
    std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point prevTime = std::chrono::system_clock::now();
    float *trakingValue;
    int16_t newValue;
    bool enStart = false;

    int8_t A;
    double M;
    float setpoint;
    uint8_t buffer[4];

public:
    double period;
    int16_t F[2] = {0};
    inline int8_t getNewValue() { return newValue; };
    PIDImpl pid = PIDImpl(period, 120, -120, 1.5, 0.1, 0);
    regulator(float *_trakingValue, double _period);
    void start(int8_t _A, double _M, float zeroAngl);
    inline void stop() {enStart = false; };
    void work();
};
