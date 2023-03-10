#pragma once
#include <Arduino.h>
#include "sensorcomm.h"
/*class DeadReckoner {
	float xpos = 0, ypos = 0, zpos = 0; // m
	float xvel = 0, yvel = 0, zvel = 0; // m^s
	float angx = 0, angy = 0, angz = 1; // unit vector pointing down
	float angr = 0; // Rotation around that axis

	void newData(RawImuData data);
}*/

namespace DeadReckoner {
	void newData(RawImuData data);
	void printData();
	void resetCalibration(); // Call when there's an IMU reset

	float getRoll();
	float getPitch();
	float getBearing();
	float getVerticalSpeed();
	float getAltitude();
	void resetPositionReckoning(float newXVel, float newYVel);
	void setGpsVertical(float altitude, float vertSpeed);
	// We want to avoid Eigen classes outside of the couple files that actually use them because of compile times
	float horizontalX();
	float horizontalY();
};
