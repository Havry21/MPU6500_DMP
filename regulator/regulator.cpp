#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "regulator.h"
#include <chrono>
#include "../I2Cdev/I2Cdev.h"
#include <cstring>
using namespace std::chrono;

regulator::regulator(float *_trakingValue, double _period) : trakingValue(_trakingValue), period(_period)
{
	pid._dt = period / 1000;
}


void regulator::work()
{
	if (enStart)
	{
		if (duration_cast<milliseconds>(system_clock::now() - prevTime).count() > period)
		{
			newValue = pid.calculate(setpoint + M, *trakingValue);
			prevTime = system_clock::now();
			F[0] = A / 2 + newValue;
			F[1] = A / 2 - newValue;
			std::memcpy(buffer, F, 4);
			// writeBytes(ARDUINO_ADR, 0x69, sizeof(buffer), buffer);
			printf("angle = %2.2f \t L = %d \t R = %d\n", *trakingValue ,F[0],F[1]);
		}
	}
}

void regulator::start(int8_t _A, double _M, float _zeroAngl)
{
	A = _A;
	M = _M;
	setpoint = _zeroAngl;
	enStart = true;
	F[0] = A / 2;
	F[1] = A / 2;
}

/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, int8_t max, int8_t min, double Kp, double Kd, double Ki) : _max(max),
																					   _min(min),
																					   _Kp(Kp),
																					   _Kd(Kd),
																					   _Ki(Ki),
																					   _pre_error(0),
																					   _integral(0)
{
	dt = _dt;
}

int16_t PIDImpl::calculate(double setpoint, double pv)
{

	// Calculate error
	double error = setpoint - pv;

	// Proportional term
	int Pout = _Kp * error;

	// Integral term
	_integral += error * _dt;
	int Iout = _Ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	int Dout = _Kd * derivative;

	// Calculate total output
	int16_t output = static_cast<int16_t>( Pout + Iout + Dout);

	// Restrict to max/min
	if (output > _max)
		output = _max;
	else if (output < _min)
		output = _min;

	// Save error to previous error
	_pre_error = error;

	return output;
}