#pragma once
constexpr int RELAY_PIN = 7;


void telemSerialSetup();
void usbSerialSetup();
void i2cSetup();
void imuSetup();

void i2cScan();

void printImuData();

struct RawImuData {
	float accelx, accely, accelz; // acceleration in g
	float gyrox, gyroy, gyroz; // speed in rad/s/
	float qw, qx, qy, qz; // DMP calculated rotation quaternion
};

RawImuData getImuData();
bool readImu();
void bumpImu();

void altimeterSetup();
void readAltimeter();

namespace airspeedCalc{

	float getAirspeed();
	void airspeedSetup();
	void readAirspeed();
};

float getBaromAltitude();
