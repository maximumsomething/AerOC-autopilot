#include <Arduino.h>
#include "inertial.h"
#include "sensorcomm.h"



void pilotsetup() {
	// todo: set pin modes
}

void pilotloop() {


	// calculate target vertical speed from target elevation
	// if within 2 meters of the desired elevation, 0
	// For the next 3 meters of error, go from 0 to (max climb rate) of desired vertical speed


	// calculate desired pitch from target vertical speed and current airspeed
	// if (current airspeed - safe airspeed) < val then calculate something from (current airspeed - safe airspeed)
	// (PI loop) - something
	// figure out the PI coefficients later

	// control elevators to set pitch

	// control throttle to set airspeed

	// control alerons to set roll (always 0 for now)

	// telemetry all control outputs and intermediate crap
}

class kpid {
	public:
	explicit kpid(float Kc, float Kp, float Ki, float Kd) : Kc(Kc), Kp(Kp), Ki(Ki), Kd(Kd){}

	float update(float newTarget, float input){
		target = newTarget;
		return update(input);
	}
	float update(float input){
		unsigned long curTime = micros();
		unsigned long dt = curTime - timeLastCalled;
		error = input - target; // calculate current error

		if(Ki != 0){
			errInt += error * (dt / 1000000.0);
		}
		if(Kd != 0){
			errDeriv = (error - prevError)/(dt / 1000000.0);
		}

		timeLastCalled = curTime;
		prevError = error;

		return Kc * target + Kp * error + Ki * errInt + Kd * errDeriv;
	}

	private:
	float Kc, Kp, Ki, Kd;
	float target;
	float error, prevError;
	float input;
	float errInt;
	float errDeriv;
	unsigned long timeLastCalled;
};