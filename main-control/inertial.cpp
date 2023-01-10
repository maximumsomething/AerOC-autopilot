#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include "telemetry_autogen.h"
#include "ringbuffer.h"
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

	// 200 Hz sample rate
	constexpr float SAMPLE_DELTA = 1.0/200.0;

	float roll = 0; //0 is level, positive is clockwise roll, negative is anticlockwise roll
	float pitch = 0; //0 is level, positive is upwards, negative is downwards
	float bearing = 0; //0 is north, values should be mod 360.

	Eigen::Quaternion<float> referenceRotation(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())); //rotation quaternion equal to what we would read when horizontal and pointed north
	Eigen::Quaternion<float> rawAttitude(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())); //current rotation quaternion
	Eigen::Quaternion<float> calibratedAttitude(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())); //current rotation quaternion
	Eigen::Vector3f accel = Eigen::Vector3f::Zero(); //current acceleration
	Eigen::Vector3f vel = Eigen::Vector3f::Zero(); //velocity
	Eigen::Vector3f pos = Eigen::Vector3f::Zero(); //position

	void newData(RawImuData data) {
		rawAttitude = Eigen::Quaternion<float>(data.qw, data.qx, data.qy, data.qz);
		calibratedAttitude = rawAttitude*referenceRotation.inverse();
		accel[0] = data.accelx;
		accel[1] = data.accely;
		accel[2] = data.accelz;
	}

	void printData() {	
		telem_pose(accel[0], accel[1], accel[2], pitch, roll, bearing);
	}

	int hasBeenStableSince = 0;
	// check whether acceleration and rotation is stable this tick
	bool checkStability() {

	}

	// maintains a ring buffer and a rolling average of acceleration data from the past second.
	void updateAverages() {

	}

	// called when we've been stable enough to calibrate
	void calibrateDown(){ //TODO: Build function to figure out which way is down
	   referenceRotation = Eigen::AngleAxisf(0, accel.normalized() - (Eigen::Vector3f::UnitZ()));
	}

	float getRoll() {return roll;}
	float getPitch() {return pitch;}
	float getBearing() {return bearing;}
}
