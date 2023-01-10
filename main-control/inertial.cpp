#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include "telemetry_autogen.h"
#include "ringbuffer.h"
#include <eigen.h>
#include <Eigen/Geometry>
#include <arduino.h>

namespace DeadReckoner {

	// 200 Hz sample rate
	constexpr float SAMPLE_DELTA = 1.0/200.0;

	float roll = 0; //0 is level, positive is clockwise roll, negative is anticlockwise roll
	float pitch = 0; //0 is level, positive is upwards, negative is downwards
	float bearing = 0; //0 is north, values should be mod 360.

	Eigen::Vector3f calibratedDown = Eigen::Vector3f::UnitZ(); //unit vector pointed down. TODO: set this to be actual down relative to the sensor board via calibration
	Eigen::Vector3f currentDown = calibratedDown;

	Eigen::Quaternion<float> attitude(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())); //current rotation quaternion
	Eigen::Vector3f vel = Eigen::Vector3f::Zero(); //velocity
	Eigen::Vector3f pos = Eigen::Vector3f::Zero(); //position

	void newData(RawImuData data) {
		attitude = Eigen::Quaternion<float>(data.qw, data.qx, data.qy, data.qz);

		currentDown = attitude._transformVector(calibratedDown);
	}

	void printData() {	
		telem_pose(currentDown.x(), currentDown.y(), currentDown.z(), pitch, roll, bearing);
	}

	void calibrateDown(); //TODO: Build funtion to figure out which way is down

	float getRoll() {return roll;}
	float getPitch() {return pitch;}
	float getBearing() {return bearing;}
}
