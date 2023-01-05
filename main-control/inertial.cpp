#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include "telemetry_autogen.h"
#include <eigen.h>
#include <Eigen/Geometry>

namespace DeadReckoner {

	// 200 Hz sample rate
	constexpr float SAMPLE_DELTA = 1.0/200.0;

	Eigen::Quaternion<float> angle(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));

	void newData(RawImuData data) {
		Eigen::Vector3f angVelVec(data.gyrox, data.gyroy, data.gyroz);
		float totalVel = angVelVec.norm() * SAMPLE_DELTA;
		//Serial.print(data.gyrox);
		//Serial.println(angVelVec.norm());
		angle = angle * Eigen::AngleAxisf(totalVel, angVelVec.normalized());
		//angle = angle * Eigen::AngleAxisf(0.001, Eigen::Vector3f::UnitX());
	}

	void printData() {
		Eigen::Vector3f down = angle._transformVector(Eigen::Vector3f::UnitZ());
		telem_downVec(down.x(), down.y(), down.z());
	}
}
