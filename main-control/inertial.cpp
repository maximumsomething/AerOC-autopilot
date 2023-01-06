#include "inertial.h"
#include "sensorcomm.h"
#include "telemetry.h"
#include "telemetry_autogen.h"
#include <eigen.h>
#include <Eigen/Geometry>

namespace DeadReckoner {

	// 200 Hz sample rate
	constexpr float SAMPLE_DELTA = 1.0/200.0;

	Eigen::Quaternion<float> angle(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	Eigen::Vector3f vel = Eigen::Vector3f::Zero();
	Eigen::Vector3f pos = Eigen::Vector3f::Zero();

	void newData(RawImuData data) {
		angle = Eigen::Quaternion<float>(data.qw, data.qx, data.qy, data.qz);
	}

	void printData() {
		Eigen::Vector3f down = angle._transformVector(Eigen::Vector3f::UnitZ());
		telem_pose(down.x(), down.y(), down.z(), vel.x(), vel.y(), vel.z(), pos.x(), pos.y(), pos.z());

	}
}
