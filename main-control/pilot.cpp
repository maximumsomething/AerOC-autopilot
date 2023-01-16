#include <Arduino.h>
#include "inertial.h"
#include "sensorcomm.h"
#include <cmath>


// constants dependent on the aircraft
constexpr float MIN_SAFE_SPEED = 5; // todo
constexpr float MAX_CLIMB_RATE = 1; // todo

// in theory could be set dynamically, but are constants right now
float targetSpeed = 10;


// set when autopilot is enabled
float targetAltitude;


// utility functions

float signf(float num) {
	if (num > 0) return 1;
	if (num < 0) return -1;
	return 0;
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

		if (firstRun) timeLastCalled = curTime;
		firstRun = false;

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
	float target = 0;
	float error = 0, prevError = 0;
	float errInt = 0;
	float errDeriv = 0;
	unsigned long timeLastCalled;
	bool firstRun = true;
};

void pilotsetup() {
	// todo: set pin modes
}

void pilotStart() {
	targetAltitude = DeadReckoner::getAltitude();
}

// calculate target vertical speed from target elevation
// piecewise linear function:
// if within 2 meters of the desired elevation, 0
// For the next 3 meters of error, go from 0 to (max climb rate) of desired vertical speed
float calcTargetVertSpeed() {
	constexpr float ELEVATION_DEADZONE = 2;
	constexpr float ELEVATION_MAX_DIFF = 5;
	float err = targetAltitude - DeadReckoner::getAltitude();
	if (fabs(err) < ELEVATION_DEADZONE) return 0;
	else {
		float amount = (fabs(err) - ELEVATION_DEADZONE) / (ELEVATION_MAX_DIFF - ELEVATION_DEADZONE);
		float mag = amount * MAX_CLIMB_RATE;
		return mag * -signf(err);
	}
}

// PID classes
kpid pitchControl(0, MAX_PITCH / MAX_CLIMB_RATE, 0, 0); // todo: figure out constants better

void pilotloop() {


	float targetVertSpeed = calcTargetVertSpeed();



	// calculate desired pitch from target vertical speed and current airspeed
	// if (current airspeed - safe airspeed) < val then calculate something from (current airspeed - safe airspeed)
	// (PI loop) - something
	// figure out the PI coefficients later

	// control elevators to set pitch

	// control throttle to set airspeed

	// control alerons to set roll (always 0 for now)

	// telemetry all control outputs and intermediate crap
}
