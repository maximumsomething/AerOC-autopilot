#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include "telemetry_autogen.h"
#include "ringbuffer.h"
#include <cmath>
#include <eigen.h>
#include <Eigen/Geometry>
#include <Arduino.h>

/*
 * Terminology:
 * Reference rotation: a quaternion equaling what the raw attitude would be when pointed horizontally and "north".
 * Raw attitude: The quaternion directly from the sensor.
 * Calibrated attitude: The raw attitude minus (i.e. multiplied by the inverse of) the reference rotation.
 * "down": 3D unit vector pointing towards the center of the earth.
 * horizontal plane: The plane orthogonal to down.
 * Pitch & roll: Derived by the offset of the current attitude from the horizontal plane.
 * Bearing: an angle from north on the horizontal plane.
 */

namespace DeadReckoner {

	using Eigen::Vector3f;
	using Eigen::Quaternionf;

	// 200 Hz sample rate
	constexpr float SAMPLE_DELTA = 1.0/200.0;

	float roll = 0; //0 is level, positive is clockwise roll, negative is anticlockwise roll
	float pitch = 0; //0 is level, positive is upwards, negative is downwards
	float bearing = 0; //0 is north, values should be mod 360.

	Quaternionf referenceRotation(Eigen::AngleAxisf(0, Vector3f::UnitZ())); //rotation quaternion equal to what we would read when horizontal and pointed north
	Quaternionf rawAttitude(Eigen::AngleAxisf(0, Vector3f::UnitZ())); //current rotation quaternion direct from the board
	Quaternionf calibratedAttitude(Eigen::AngleAxisf(0, Vector3f::UnitZ())); //current rotation quaternion, relative to the reference rotation
	Vector3f rawAccel = Vector3f::Zero(); //current rawAcceleration direct from the sensor
	Vector3f calibratedAccel= Vector3f::Zero(); //current rawAcceleration multiplied by calibrated attitude

	constexpr int samplesToCalibrate = 200; // one second
	int stableSamples = 0;

	float inertialVerticalSpeed = 0;
	float prevBaromAltitude= 0;
	float baromVerticalSpeed = 0;
	ring_buffer<float> dVertSpeedBuf(1000, 0);
	float dVertSpeedAvg;
	float verticalSpeed = 0;

//#define CALIBRATE_ACCEL_BIAS

	// check whether acceleration and rotation is stable this sample
	bool checkStability(const RawImuData& data) {
#ifdef CALIBRATE_ACCEL_BIAS
		constexpr float maxDps = 5;
		constexpr float maxGDiff = 0.2;
#else
		constexpr float maxDps = 0.5;
		constexpr float maxGDiff = 0.03;
#endif
		bool gyroStable = fabs(data.gyrox) < maxDps && fabs(data.gyroy) < maxDps && fabs(data.gyroz) < maxDps;
		if (!gyroStable) return false;
		// check magnitude of acceleration is 1g
		float rawAccel = Vector3f(data.accelx, data.accely, data.accelz).norm();
		//Serial.printf("rawAccel: %f\n", rawAccel);
		return fabs(rawAccel - 1) < maxGDiff;
	}

	constexpr int SAMPLES_TO_AVERAGE = 200;
	ring_buffer<Vector3f> premultipliedPastAccels(SAMPLES_TO_AVERAGE, Vector3f::Zero());

	// maintains a ring buffer and a rolling average of acceleration data from the past second.
	Vector3f averageAccel = Vector3f::Zero();
	void updateAverages(Vector3f rawAccel) {
		Vector3f multAccel = rawAccel * (1.0 / SAMPLES_TO_AVERAGE);
		averageAccel += multAccel;
		averageAccel -= premultipliedPastAccels.pop();
		premultipliedPastAccels.put(multAccel);
	}

#ifdef CALIBRATE_ACCEL_BIAS

	float calPosXAccelBias(Vector3f posXAccel);
	Vector3f rawAccelBias(nanf(""), nanf(""), nanf(""));
	bool biasCalibrated[6] = { false, false, false, false, false, false };

	// Call when stable to update the bias for the current down axis.
	void calibrateAccelBias() {
		//Serial.printf("calibrateAccelBias with x=%f y=%f z=%f\n", averageAccel[0], averageAccel[1], averageAccel[2]);

		Vector3f posXAccel = averageAccel;
		float sign = 1;

		for (int i = 0; i < 6; ++i) {

			if (!biasCalibrated[i]) {
				float bias = calPosXAccelBias(posXAccel * sign);
				if (!isnanf(bias)) {

					if (isnanf(rawAccelBias[i/2])) rawAccelBias[i/2] = bias * sign;
					else rawAccelBias[i/2] = rawAccelBias[i/2] * 0.5 + bias * sign * 0.5;

					Serial.printf("***Calibrated bias for axis %d (%c%c)***\n\n\n\n",
								  i,
								  (sign == 1) ? '+': '-',
								  'x' + i/2);
					biasCalibrated[i] = true;
				}
			}

			if (sign == 1) sign = -1;
			else {
				sign = 1;
				// swap axes
				float x = posXAccel[0];
				posXAccel[0] = posXAccel[1];
				posXAccel[1] = posXAccel[2];
				posXAccel[2] = x;
			}
		}

		bool allAxesCalibrated = true;
		for (int i = 0; i < 6; ++i) allAxesCalibrated = allAxesCalibrated && biasCalibrated[i];
		if (allAxesCalibrated) {
			Serial.printf("Calibrated rawAccelerometer biases: x=%f y=%f z=%f\n", rawAccelBias[0], rawAccelBias[1], rawAccelBias[2]);
		}
	}
	// If the given vector is pointing along the +X axis, return the bias.
	float calPosXAccelBias(Vector3f posXAccel) {
		constexpr float MAX_UNCAL_ACCEL_DIFF = 0.2;

		if (posXAccel[0] > 0
			&& fabs(posXAccel[0] - 1.0) < MAX_UNCAL_ACCEL_DIFF
			&& fabs(posXAccel[1]) < MAX_UNCAL_ACCEL_DIFF
			&& fabs(posXAccel[2]) < MAX_UNCAL_ACCEL_DIFF) {

			return 1.0 - posXAccel[0];
		}
		else return nanf("");
	}
#endif

	void calibrateDown();

	void newData(RawImuData data) {
		rawAttitude = Quaternionf(data.qw, data.qx, data.qy, data.qz);
		calibratedAttitude = rawAttitude*referenceRotation.inverse();
		rawAccel[0] = data.accelx;
		rawAccel[1] = data.accely;
		rawAccel[2] = data.accelz;

		calibratedAccel = calibratedAttitude._transformVector(rawAccel);

		Vector3f down = calibratedAttitude._transformVector(Vector3f::UnitZ());

		pitch = -atan(down[0]/down[2])*(180.0/M_PI); //c++ trig builtins output in radians, convert to degrees
		roll = -atan(down[1]/down[2])*(180.0/M_PI);
		//if(down[2] > 0){
		//	roll += 90;
		//}

		Vector3f forward = calibratedAttitude._transformVector(Vector3f::UnitX());
		bearing = -atan2(forward[1], forward[0])*(180.0/M_PI) + 180;


		updateAverages(rawAccel);

		if (checkStability(data)) {
			stableSamples++;
		}
		else stableSamples = 0;
		if (stableSamples >= samplesToCalibrate) {
			//Serial.println("Stable! calibrating");
			calibrateDown();
#ifdef CALIBRATE_ACCEL_BIAS
			calibrateAccelBias();
#endif
		}
	}

	void calcVerticalSpeed(){
		float curBaromAltitude = getBaromAltitude(); //calculate barometric vertical speed
		baromVerticalSpeed = curBaromAltitude - prevBaromAltitude;
		prevBaromAltitude = curBaromAltitude;

		inertialVerticalSpeed += calibratedAccel[2];

		float dVertSpeed = baromVerticalSpeed - inertialVerticalSpeed;
		dVertSpeedAvg += dVertSpeed * (1/(float)dVertSpeedBuf.capacity());

		if(dVertSpeedBuf.full()){
			dVertSpeedAvg -= dVertSpeedBuf.pop()*.001;
		}

		verticalSpeed = inertialVerticalSpeed + dVertSpeedAvg;
	}

	void printData() {	
		telem_pose(rawAccel[0], rawAccel[1], rawAccel[2], pitch, roll, bearing);
	}

	// called when we've been stable enough to calibrate
	void calibrateDown(){ //TODO: Build function to figure out which way is down
	   referenceRotation = rawAttitude*Quaternionf::FromTwoVectors(accel, Vector3f::UnitZ());
	   // light up onboard LED when calibrated
	   digitalWrite(13, HIGH);
	}

	float getRoll() {return roll;}
	float getPitch() {return pitch;}
	float getVerticalSpeed() {return verticalSpeed;}
}
