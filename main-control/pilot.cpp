#include <Arduino.h>
#include "pilot.h"
#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include <cmath>
#include <PWMServo.h>


// constants dependent on the aircraft
constexpr float MIN_SAFE_AIRSPEED = 4;
constexpr float AIRSPEED_CORRECTION_START = 6;
constexpr float AIRSPEED_CORRECTION_FACTOR = 30 / (AIRSPEED_CORRECTION_START - MIN_SAFE_AIRSPEED); // degrees per (m/s)

constexpr float MAX_CLIMB_RATE = 1; // conservative

constexpr float MIN_PITCH = -30; // degrees
constexpr float MAX_PITCH = 30; // degrees

constexpr float TOP_SPEED = 12; // Theoretical top airspeed used for calculating throttle

constexpr bool TEST_MODE = false; //Test mode, disables throttle if true

// in theory could be set dynamically, but are constants right now
float targetSpeed = 8;
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
	kpid(float min, float max, float Kc, float Kp, float Ki, float Kd) : outMin(min), outMax(max), Kc(Kc), Kp(Kp), Ki(Ki), Kd(Kd), intBound((max - min) / Ki) {}

	kpid(float min, float max, float Kc, float Kp, float Ki, float Kd, float outBound) : kpid(min, max, Kc, Kp, Ki, Kd) {
		intBound = outBound / Ki;
	}

	float update(float newTarget, float input){
		target = newTarget;
		return update(input);
	}
	float update(float input){
		unsigned long curTime = micros();

		unsigned long dt = curTime - timeLastCalled;
		error = input - target; // calculate current error

		if (!firstRun) {
			if(Ki != 0){
				errInt += error * (dt / 1000000.0);
				if((intBound != 0) && (fabs(errInt) > intBound)){
					errInt = intBound * signf(errInt);
				}
			}
			if(Kd != 0){
				errDeriv = (error - prevError)/(dt / 1000000.0);
			}
		}

		timeLastCalled = curTime;
		prevError = error;
		firstRun = false;

		float outVal = Kc * target + Kp * error + Ki * errInt + Kd * errDeriv;
		if (outVal < outMin) outVal = outMin;
		if (outVal > outMax) outVal = outMax;
		return outVal;
	}

	private:
	float outMin, outMax;
	float Kc, Kp, Ki, Kd;
	unsigned long timeLastCalled;
	float target = 0;
	float error = 0, prevError = 0;
	float errInt = 0;
	float errDeriv = 0;
	float intBound = 0;
	bool firstRun = true;
};

//Servo control instances
PWMServo aileronServo; //Handlers for control axes should always be declared in the order they are arranged on the receiver - Ailerons/Roll, Elevator/Pitch, Throttle/Speed, Rudder/Yaw
PWMServo elevatorServo;
PWMServo throttleServo;
PWMServo rudderServo;

void pilotSetup() {
	aileronServo.attach(2, 1000, 2000);
	elevatorServo.attach(3, 1000, 2000);
	throttleServo.attach(4, 1000, 2000);
	rudderServo.attach(5, 1000, 2000);
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
		amount = fmin(amount, 1);
		float mag = amount * MAX_CLIMB_RATE;
		return mag * -signf(err);
	}
}

int pilotLastPrintTime = 0;

// PID classes
kpid aileronControl(-1, 1, 0, 1.0/30.0, .25 / ((30.0 * (1.0/3.0)) * 2.0 / 2.0), 0, 0.1);
kpid pitchControl(MIN_PITCH, MAX_PITCH, 0, MAX_PITCH / MAX_CLIMB_RATE * 0.5, 0, 0, 10); // todo: figure out constants better
// kp: estimated by manual pilot
// ki: We want to reach an integral term of 1/3 within 2 seconds
// ends up being: 1 / ((1/2) * seconds * desiredTerm * (1/kp))
kpid elevatorControl(-1, 1, 0, 1.0/30.0, 1.0 / ((30.0 * (1.0/3.0)) * 2.0 * (1.0 / 2.0)), 0, 0.3);
// just kinda guessing at good constants here
kpid throttleControl(0, 1, 1 / TOP_SPEED, 0.7 / TOP_SPEED, 0, 0);

void pilotLoop() {
	//const float targetVertSpeed = calcTargetVertSpeed();
	constexpr float targetVertSpeed = 0;

	// calculate desired pitch from target vertical speed and current airspeed
	// if (current airspeed - safe airspeed) < val then calculate something from (current airspeed - safe airspeed)
	// (PI loop) - something
	// figure out the PI coefficients later
	float targetPitch = -pitchControl.update(targetVertSpeed, DeadReckoner::getVerticalSpeed());
	float airspeed = airspeedCalc::airspeed;
	if (airspeed < AIRSPEED_CORRECTION_START && !TEST_MODE) {
		targetPitch -= AIRSPEED_CORRECTION_FACTOR * (AIRSPEED_CORRECTION_START - airspeed);
	}
	if (targetPitch < MIN_PITCH) targetPitch = MIN_PITCH;

	// control elevators to set pitch
	float elevatorSignal = -elevatorControl.update(targetPitch, DeadReckoner::getPitch());

	// control throttle to set airspeed
	float throttleSignal = throttleControl.update(targetSpeed, airspeed);
	
	// control aileron to set roll (always 0 for now)
	float aileronSignal = aileronControl.update(0, DeadReckoner::getRoll());

	//TODO - yaw

	if (millis() - pilotLastPrintTime >= 200) {
		telem_controlOut(targetVertSpeed, targetPitch, throttleSignal, elevatorSignal, aileronSignal);
		pilotLastPrintTime = millis();
	}


	//all control outputs and intermediate crap
	aileronSignal = (aileronSignal * 90) + 90;
	elevatorSignal = (elevatorSignal * 70) + 90;
	if(std::isnan(throttleSignal)) throttleSignal = 110;
	else throttleSignal *= 180;

	aileronServo.write(aileronSignal);
	elevatorServo.write(elevatorSignal);
	if(!TEST_MODE) throttleServo.write(throttleSignal);
	else throttleServo.write(0);
	rudderServo.write(90);
}
