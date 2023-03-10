#include <Arduino.h>
#include "pilot.h"
#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include "gpsnav.h"
#include "telemetry_autogen.h"
#include <cmath>
#include <PWMServo.h>


// utility functions. Maybe put these in their own file at some point

float signf(float num) {
	if (num > 0) return 1;
	if (num < 0) return -1;
	return 0;
}

// returns a value of s - t between -180 and 180 degrees.
float normAngle(float x) {
	float ret = fmod(x, 360);
	if (ret < -180) ret += 360;
	if (ret > 180) ret -= 360;
	return ret;
}

float angleDiff(float s, float t) {
	return normAngle(s - t);
}

namespace Pilot {

// constants dependent on the aircraft
constexpr float MIN_SAFE_AIRSPEED = 4;
constexpr float AIRSPEED_CORRECTION_START = 6;
constexpr float AIRSPEED_CORRECTION_FACTOR = 30 / (AIRSPEED_CORRECTION_START - MIN_SAFE_AIRSPEED); // degrees per (m/s)

constexpr float TOP_SPEED = 12; // Theoretical top airspeed used for calculating throttle

constexpr float MAX_CLIMB_RATE = 3.5; // Maximum vertical speed the autopilot will try for
constexpr float STANDARD_CLIMB_RATE = 2; // Reasonable vertical speed that some parameters are calculated from

constexpr float MIN_PITCH = -30; // degrees
constexpr float MAX_PITCH = 30; // degrees
constexpr float DEFAULT_PITCH = 5; // Target pitch when altitude control is disabled

constexpr float MAX_ROLL = 30;

constexpr float RUDDER_MAXTHROW = 0.6;
constexpr float ELEVATOR_MAXTHROW = 0.8;
constexpr float AILERON_MAXTHROW = 1;
constexpr float THROTTLE_MAXTHROW = 1;


constexpr float SAFETY_MARGIN = 1.5; //the ammount that we can exceed max roll or pitch by before entering recovery mode

constexpr float DIRECTION_STICKY_MARGIN = 20; // If we are within this angle of 180deg from the target, don't switch directions.

constexpr bool TEST_MODE = false; //Test mode, disables throttle if true
//#define NO_PILOT_START // Defined when there is no way of knowing when the autopilot starts


targets_t targets;


bool unsafeRegime = false; //enabled if max pitch or roll is exceeded by the safety margin, disables setTargetBearing() and sets target pitch to _
float prevBearingDiff = 0; // Used for direction stickiness

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
	//targets.altitude = DeadReckoner::getAltitude();
	//targets.bearing = DeadReckoner::getBearing();
}


// calculate target vertical speed from target elevation
// piecewise linear function:
// if within 4 meters of the desired elevation, 0
// For the next 8 meters of error, go from 0 to (max climb rate) of desired vertical speed
float calcTargetVertSpeed() {
	constexpr float ELEVATION_DEADZONE = 4;
	constexpr float ELEVATION_MAX_DIFF = 8;
	float err = DeadReckoner::getAltitude() - targets.altitude;
	if (fabs(err) < ELEVATION_DEADZONE) return 0;
	else {
		float amount = (fabs(err) - ELEVATION_DEADZONE) / (ELEVATION_MAX_DIFF - ELEVATION_DEADZONE);
		amount = fmin(amount, 1);
		float mag = amount * MAX_CLIMB_RATE;
		return mag * -signf(err);
	}
}

//unsigned long pilotLastPrintTime = 0;

// PID classes
// the correct constant is on the order of magnitude of 1, so why not just have it be 1?
//kpid rollControl(MAX_ROLL * -1, MAX_ROLL, 0, 1, 0, 0);
// The integral term here is just a quarter of the elevator integral term
kpid aileronControl(-1, 1, 0, 1.0/30.0, .25 / ((30.0 * (1.0/3.0)) * 2.0 / 2.0), 0, 0.1);

kpid pitchControl(MIN_PITCH, MAX_PITCH,
				  // Keep the feed-forward term small because we don't know very well what it should be
				  MAX_PITCH / STANDARD_CLIMB_RATE * 0.15,
				  // Keep the proportional term also kinda small because the integral term is doing most of the work
				  MAX_PITCH / STANDARD_CLIMB_RATE * 0.5,
				  // Want to reach an integral term of 20 degrees within 3 seconds
				  1.0 / ((STANDARD_CLIMB_RATE/MAX_PITCH * 20) * 3.0 * (1.0 / 2.0)), 0, 20);
// kp: estimated by manual pilot
// ki: We want to reach an integral term of 1/3 within 2 seconds
// ends up being: 1 / ((1/2) * seconds * desiredTerm * (1/kp))
kpid elevatorControl(-1, 1, 0, 1.0/30.0, 1.0 / ((30.0 * (1.0/3.0)) * 2.0 * (1.0 / 2.0)), 0, 0.3);
// just kinda guessing at good constants here
kpid throttleControl(0, 1, 1 / TOP_SPEED, -0.7 / TOP_SPEED, 0, 0);
//Constants determined by vibes

void pilotLoop() {

	float airspeed = airspeedCalc::getAirspeed();

	float targetPitch = DEFAULT_PITCH;
	float targetVertSpeed = NAN;
	if (!isnanf(targets.altitude)) {

#ifdef NO_PILOT_START
		targetVertSpeed = 0;
#else
		targetVertSpeed = calcTargetVertSpeed();
#endif
		if((fabs(DeadReckoner::getRoll()) > MAX_ROLL * SAFETY_MARGIN
			|| DeadReckoner::getPitch() < MIN_PITCH * SAFETY_MARGIN)
			|| (airspeed != 0 && airspeed < MIN_SAFE_AIRSPEED / SAFETY_MARGIN)) {
			if (!unsafeRegime) telem_warningFlightRegime(unsafeRegime);
			unsafeRegime = true;
		}
		if(unsafeRegime &&
			fabs(DeadReckoner::getRoll()) < MAX_ROLL
			&& DeadReckoner::getPitch() > MIN_PITCH
			&& ((airspeed == 0 || isnanf(airspeed)) || airspeed > MIN_SAFE_AIRSPEED)){
			if (unsafeRegime) telem_warningFlightRegime(unsafeRegime);
			unsafeRegime = false;
		}

		// calculate desired pitch from target vertical speed and current airspeed
		// if (current airspeed - safe airspeed) < val then calculate something from (current airspeed - safe airspeed)
		// (PI loop) - something
		// figure out the PI coefficients later
		targetPitch = -pitchControl.update(targetVertSpeed, DeadReckoner::getVerticalSpeed());
		if (airspeed != 0 && airspeed < AIRSPEED_CORRECTION_START && !TEST_MODE) {
			targetPitch -= AIRSPEED_CORRECTION_FACTOR * (AIRSPEED_CORRECTION_START - airspeed);
		}
		if (targetPitch < MIN_PITCH) targetPitch = MIN_PITCH;
	}

	// control elevators to set pitch
	float elevatorSignal = -elevatorControl.update(targetPitch, DeadReckoner::getPitch());

	float throttleSignal = 0.6;
	if (airspeed != 0 && !isnanf(airspeed) && !isnanf(targets.airspeed)) {
		// control throttle to set airspeed
		throttleSignal = throttleControl.update(targets.airspeed, airspeed);
	}
	
	float targetRoll = 0;
	float rudderSignal = 0;
	if (!isnanf(targets.bearing)) {
		// control aileron to set roll
		// Calculate the angular difference from the target (from -180 to 180)
		float bearingDiff = angleDiff(DeadReckoner::getBearing(), targets.bearing);

		// If we crossed the 180 degree line, don't. Instead, just make bearingDiff go outside of the -180 to 180 range.
		if (fabs(bearingDiff) > 180 - DIRECTION_STICKY_MARGIN
			&& signf(bearingDiff) != signf(prevBearingDiff)) {

			if (bearingDiff > 0) bearingDiff -= 360;
			else bearingDiff += 360;
		}
		prevBearingDiff = bearingDiff;


		// don't use kpid class, because we don't need i and d terms and we have a difference not a target and input
		// the correct kP is on the order of magnitude of 1, so why not just have it be 1?
		if(!unsafeRegime){
			targetRoll = -1 * bearingDiff;
			targetRoll = fmax(targetRoll, -MAX_ROLL);
			targetRoll = fmin(targetRoll, MAX_ROLL);
		}else{
			targetRoll = 0;
		}

		//Constants determined by vibes
		if(!unsafeRegime){
			rudderSignal = -(1.0/30.0) * bearingDiff;
			rudderSignal = fmax(rudderSignal, -1);
			rudderSignal = fmin(rudderSignal, 1);
		} else {
			rudderSignal = 0;
		}
	}

	float aileronSignal = aileronControl.update(targetRoll, DeadReckoner::getRoll());

	//if (millis() - pilotLastPrintTime >= 200) {
		telem_controlOut(targetVertSpeed, targets.altitude, targetPitch, targetRoll, targets.bearing, throttleSignal, elevatorSignal, aileronSignal, rudderSignal);
		//pilotLastPrintTime = millis();
	//}

	if(unsafeRegime && DeadReckoner::getPitch() < MIN_PITCH * 2){
		aileronSignal = 0;
		rudderSignal = 0;
	}

	// Translate control outputs from -1 to 1 to the degree values that PWMServo expects
	aileronSignal = (aileronSignal * AILERON_MAXTHROW * 90) + 90;
	elevatorSignal = (elevatorSignal * ELEVATOR_MAXTHROW * 90) + 90;
	rudderSignal = (rudderSignal * RUDDER_MAXTHROW * 90) + 90;
	throttleSignal *= THROTTLE_MAXTHROW * 180;


	aileronServo.write(aileronSignal);
	elevatorServo.write(elevatorSignal);
	if(!TEST_MODE) throttleServo.write(throttleSignal);
	else throttleServo.write(0);
	rudderServo.write(rudderSignal);
}
String clue = "If you want to reach the stars, you should find the rules for flying. 2 Rotor II St A Init B";
}
