

#include "telemetry.h"
#include "sensorcomm.h"
#include "inertial.h"
#include "ringbuffer.h"

void setup() {
	setupAllComms();
}

// 200Hz
constexpr int loopInterval = 5000; // microseconds

int lastPrintTime = 0;
int loopCounter = 0;

int successfulImuReads = 0;

void loop() {
	++loopCounter;
	//telem_fakedata(123, 4567);
	/*telem_morefakestuff(3.2, 1000000, 2.1673453459345);
	telem_strmessage("blahblah");
	//Serial.print("garbage");
	delay(1000);*/
	//printImuData();


	int startTime = micros();

	// readImu will return true as long as there was new data to be read.
	int readCounter = 0;
	RawImuData imuData;
	while (readImu()) {
		++readCounter;
		++successfulImuReads;
		imuData = getImuData();
		DeadReckoner::newData(imuData);
	}
  airspeedCalc::readAirspeed();
	/*if (readCounter != 1) {
		Serial.printf("Fascinating, we read %d imu values on this loop\n", readCounter);
	}*/
	if (readCounter == 0) {
		if (successfulImuReads != 0) {
			bumpImu();
			Serial.printf("IMU bus reset after %d successful reads\n", successfulImuReads);
			successfulImuReads = 0;
		}
		else {
			telem_strmessage("WARN: reset IMU\n\n\n");
			imuSetup();
		}
	}

	if (startTime > lastPrintTime + 200000) {
		if (readCounter > 0) {
			DeadReckoner::printData();
			lastPrintTime = startTime;
			//Serial.printf("ax=%f, ay=%f, az=%f\n", imuData.accelx, imuData.accely, imuData.accelz);
			//Serial.printf("gx=%f, gy=%f, gz=%f\n", imuData.gyrox, imuData.gyroy, imuData.gyroz);
		}
	}
	// Do this at 25 Hz
	if (loopCounter % 8 == 0) {
		readAltimeter();
	}
	int endTime = micros();
	// make loop
	int delayTime = loopInterval - (endTime - startTime);
	if (delayTime < 0) {
		// Serial.print("Warning: loop ran over by "); Serial.print(-delayTime); Serial.println(" microseconds");
	}
	else delayMicroseconds(delayTime);
}
