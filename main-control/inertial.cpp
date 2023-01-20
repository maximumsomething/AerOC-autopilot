#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include "telemetry_autogen.h"
#include "ringbuffer.h"
#include <cmath>
#include <eigen.h>
#include <Eigen/Geometry>

/*
 * Terminology:
 * Reference rotation: a quaternion equaling what the raw attitude would be when pointed horizontally and "north".
 * Raw attitude: The quaternion directly from the sensor.
 * Calibrated attitude: The raw attitude minus (i.e. multiplied by the inverse of) the reference rotation.
 * Calibrated acceleration: The accelerometer output transformed by the calibrated attitude.
 * "down": 3D unit vector pointing towards the center of the earth.
 * horizontal plane: The plane orthogonal to down.
 * Pitch & roll: Derived by the offset of the current attitude from the horizontal plane.
 * Bearing: an angle from north on the horizontal plane.
 *
 * Directions:
 * For calibratedAccel, up is +z, forward is +x, and left is +y.
 * For pitch, up is positive and down is negative.
 * For roll, left (CCW) is negative and right (CW) is positive.
 * For bearing, it is like a compass heading: right (clockwise) is positive when looking down.
 */

namespace DeadReckoner {

	using Eigen::Vector3f;
	using Eigen::Quaternionf;

	// 200 Hz sample rate
	constexpr float SAMPLE_DELTA = 1.0/200.0; // seconds

	float roll = 0; //0 is level, positive is clockwise roll, negative is anticlockwise roll
	float pitch = 0; //0 is level, positive is upwards, negative is downwards
	float bearing = 0; //0 is north, values should be mod 360.

	Quaternionf referenceRotation = Quaternionf::Identity(); //rotation quaternion equal to what we would read when horizontal and pointed north
	Quaternionf rawAttitude = Quaternionf::Identity(); //current rotation quaternion direct from the board
	Quaternionf calibratedAttitude = Quaternionf::Identity(); //current rotation quaternion, relative to the reference rotation
	Vector3f rawAccel = Vector3f::Zero(); //current rawAcceleration direct from the sensor
	Vector3f calibratedAccel= Vector3f::Zero(); //current rawAcceleration multiplied by calibrated attitude
	float calibratedG = 1.0; // gravity, should be very close to 1g

	constexpr int samplesToCalibrate = 400; // two seconds
	int stableSamples = 0;

	float prevBaromAltitude = 0;

	bool downCalibrated = false;

	int samplesGotten = 0;
	Vector3f startupGyroAvg = Vector3f::Zero();


//#define CALIBRATE_ACCEL_BIAS

	// check whether acceleration and rotation is stable this sample
	bool checkStability(const RawImuData& data) {
#ifdef CALIBRATE_ACCEL_BIAS
		constexpr float maxDps = 5;
		constexpr float maxGDiff = 0.2;
#else
		constexpr float maxDps = 0.5;
		//constexpr float maxGDiff = 0.05;
		constexpr float maxGDiff = 0.08;
#endif
		bool gyroStable = fabs(data.gyrox) < maxDps && fabs(data.gyroy) < maxDps && fabs(data.gyroz) < maxDps;
		if (!gyroStable) return false;
		// check magnitude of acceleration is 1g
		float rawAccel = Vector3f(data.accelx, data.accely, data.accelz).norm();
		//Serial.printf("rawAccel: %f\n", rawAccel);
		return fabs(rawAccel - 1) < maxGDiff;
	}

	constexpr int SAMPLES_TO_AVERAGE = 400;
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


	// drift corrected integrator
	class DriftCorrInt {
		float timeDelta;
		// Every iteration, new samples are multiplied by this (and old samples by 1 - this)
		float multiplier;

		//float inertialVal;
		//float drift;
	public:
		float lastVal;
		// tau is the mean life of a sample
		// At time t, the sample at t1 is weighted by e^(t-t1)/tau
		DriftCorrInt(float timeDelta, float tau, float initVal = 0):
			timeDelta(timeDelta),
			multiplier(1 - exp(-timeDelta/tau)),
			lastVal(initVal)
			{}

		float newVal(float toIntegrate, float noisyTarget) {
			//inertialVal += toIntegrate * timeDelta;
			//drift = drift * (1 - multiplier) + (noisyTarget - inertialVal) * multiplier;
			//return inertialVal + drift;
			lastVal += toIntegrate * timeDelta;
			lastVal += multiplier * (noisyTarget - lastVal);
			return lastVal;
		}
	};

	DriftCorrInt verticalSpeedCalculator(SAMPLE_DELTA, 20);
	DriftCorrInt altitudeCalculator(SAMPLE_DELTA, 5);

	void calibrateDown();

	void newData(RawImuData data) {
		rawAttitude = Quaternionf(data.qw, data.qx, data.qy, data.qz);
		calibratedAttitude = referenceRotation.inverse()*rawAttitude;
		rawAccel = Vector3f(data.accelx, data.accely, data.accelz);

		calibratedAccel = calibratedAttitude._transformVector(rawAccel);

		//Serial.printf("calibratedAttitude: %f\n", calibratedAttitude.angularDistance(Quaternionf::Identity()) * (180 / M_PI));

		//Vector3f down = calibratedAttitude._transformVector(Vector3f::UnitZ());
		Vector3f forward = calibratedAttitude._transformVector(Vector3f::UnitX());
		Vector3f left = calibratedAttitude._transformVector(Vector3f::UnitY());

		//Serial.printf("forward: x=%f, y=%f, z=%f\n", forward.x(), forward.y(), forward.z());

		pitch = -asin(forward.z())*(180.0/M_PI);
		roll = -asin(left.z())*(180.0/M_PI);
		bearing = -atan2(forward[1], forward[0])*(180.0/M_PI) + 180;


		updateAverages(rawAccel);

		// calculate vertical speed & altitude with barometer & accelerometer
		float curBaromAltitude = getBaromAltitude();
		float baromVerticalSpeed = (curBaromAltitude - prevBaromAltitude) / SAMPLE_DELTA;
		prevBaromAltitude = curBaromAltitude;

		// Subtract gravity, convert to m/s^2
		float accelms = (calibratedAccel[2] - calibratedG) * 9.80665;

		// integrate and correct for drift
		float verticalSpeed = verticalSpeedCalculator.newVal(accelms, baromVerticalSpeed);
		altitudeCalculator.newVal(verticalSpeed, curBaromAltitude);


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

		++samplesGotten;
		// average gyro values for calibration purposes
		constexpr int GYRO_SAMPLES_TO_AVERAGE = 1000; // 5 seconds, so it will end before automatic calibration starts
		startupGyroAvg += Vector3f(data.gyrox, data.gyroy, data.gyroz) * (1.0 / GYRO_SAMPLES_TO_AVERAGE);
		if (samplesGotten == GYRO_SAMPLES_TO_AVERAGE) {
			Serial.printf("Startup gyro average: x=%f y=%f z=%f\n\n", startupGyroAvg.x(), startupGyroAvg.y(), startupGyroAvg.z());
		}
	}

	void printData() {
		telem_calInertial(calibratedAccel.x(), calibratedAccel.y(), calibratedAccel.z() - calibratedG, calibratedAccel.norm());
		telem_pose(pitch, roll, bearing, getVerticalSpeed(), getAltitude());
	}

	// called when we've been stable enough to calibrate
	void calibrateDown(){ //TODO: Build function to figure out which way is down
		//if (!downCalibrated) {
			calibratedG = averageAccel.norm();
			Quaternionf angleFromDown = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), averageAccel);
			referenceRotation = rawAttitude*angleFromDown;

			// reset vertical drift
			verticalSpeedCalculator.lastVal = 0;
			altitudeCalculator.lastVal = getBaromAltitude();

			// light up onboard LED when calibrated
			digitalWrite(13, HIGH);

		// //}
		if (!downCalibrated) Serial.printf("***Calibrated down: calibratedG=%f, angleFromDown=%f\n\n\n", calibratedG, angleFromDown.angularDistance(Quaternionf::Identity()) * 180 / M_PI);

		downCalibrated = true;
	}

	void resetCalibration() {
		referenceRotation = Quaternionf::Identity();
		digitalWrite(13, LOW);
	}

	float getRoll() {return roll;}
	float getPitch() {return pitch;}
	float getBearing() {return bearing;}
	float getVerticalSpeed() { return verticalSpeedCalculator.lastVal; }
	float getAltitude() { return altitudeCalculator.lastVal; }
}
