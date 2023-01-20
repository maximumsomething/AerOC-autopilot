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

constexpr float MAX_ROLL = 30;

constexpr float TOP_SPEED = 12; // Theoretical top airspeed used for calculating throttle

constexpr bool TEST_MODE = false; //Test mode, disables throttle if true
#define NO_PILOT_START // Defined when there is no way of knowing when the autopilot starts

// in theory could be set dynamically, but are constants right now
float targetSpeed = 8;
// set when autopilot is enabled. Unused if NO_PILOT_START.
float targetAltitude;
// set when the autopilot is enabled.
float targetBearing = 180;

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
// Positive rudder values move the rudder right, turning the plane right
// Positive elevator values push the elevator up, turning the plane up
// Positive aileron values roll the left wing down (roll it CCW).

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
	targetBearing = DeadReckoner::getBearing();
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

unsigned long pilotLastPrintTime = 0;

// PID classes
// the correct constant is on the order of magnitude of 1, so why not just have it be 1?
//kpid rollControl(MAX_ROLL * -1, MAX_ROLL, 0, 1, 0, 0);
kpid aileronControl(-1, 1, 0, 1.0/30.0, .25 / ((30.0 * (1.0/3.0)) * 2.0 / 2.0), 0, 0.1);
// todo: figure out constants better
kpid pitchControl(MIN_PITCH, MAX_PITCH, 0, MAX_PITCH / MAX_CLIMB_RATE * 0.5, 0, 0, 10);
// kp: estimated by manual pilot
// ki: We want to reach an integral term of 1/3 within 2 seconds
// ends up being: 1 / ((1/2) * seconds * desiredTerm * (1/kp))
kpid elevatorControl(-1, 1, 0, 1.0/30.0, 1.0 / ((30.0 * (1.0/3.0)) * 2.0 * (1.0 / 2.0)), 0, 0.3);
// just kinda guessing at good constants here
kpid throttleControl(0, 1, 1 / TOP_SPEED, 0.7 / TOP_SPEED, 0, 0);
//Constants determined by vibes

void pilotLoop() {
#ifdef NO_PILOT_START
	constexpr float targetVertSpeed = 0;
#else
	const float targetVertSpeed = calcTargetVertSpeed();
#endif

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
	
	// control aileron to set roll
	//float targetRoll = rollControl.update(targetBearing, DeadReckoner::getBearing());
	// Calculate the angular difference from the target
	float bearingDiff = DeadReckoner::getBearing() - targetBearing;
	if (bearingDiff < -180) bearingDiff += 360;
	if (bearingDiff > 180) bearingDiff -= 360;

	// don't use kpid class, because we don't need i and d terms and we have a difference not a target and input
	// the correct kP is on the order of magnitude of 1, so why not just have it be 1?
	float targetRoll = 1 * bearingDiff;
	targetRoll = fmax(targetRoll, -MAX_ROLL);
	targetRoll = fmin(targetRoll, MAX_ROLL);

	float aileronSignal = -aileronControl.update(targetRoll, DeadReckoner::getRoll());

	//float rudderSignal = rudderControl.update(targetBearing, DeadReckoner::getBearing());
	//Constants determined by vibes
	float rudderSignal = -(1.0/30.0) * bearingDiff;
	rudderSignal = fmax(rudderSignal, -1);
	rudderSignal = fmin(rudderSignal, 1);
	//float rudderSignal = 0;

	// aircraft specific rudder settings
	rudderSignal *= 0.6;


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
	rudderServo.write((rudderSignal * 90) + 90);
}
