

#include "telemetry.h"
#include "sensorcomm.h"
#include "inertial.h"
#include "pilot.h"

void setup() {
	setupAllComms();
	pilotSetup();
	telem_strmessage("Initialization complete");
}

// 200Hz
constexpr int loopInterval = 20000; // microseconds

int lastPrintTime = 0;
int loopCounter = 0;

int totalImuReads = 0;
int ticksSinceLastImuRead = 0;

void loop() {
	++loopCounter;

	int startTime = micros();

	// readImu will return true as long as there was new data to be read.
	int tickImuReads = 0;
	RawImuData imuData;
	while (readImu()) {
		//Serial.print(micros() - startTime); Serial.print(" ");
		++tickImuReads;
		++totalImuReads;
		imuData = getImuData();
		DeadReckoner::newData(imuData);
		//Serial.println(micros() - startTime);
		if (tickImuReads > 4) {
			break;
		}
	}
	//Serial.println(micros() - startTime);
	//Serial.printf("We read %d imu values on this loop\n", tickImuReads);
	if (tickImuReads == 0) {
		ticksSinceLastImuRead++;
		if (ticksSinceLastImuRead >= 50) {
			telem_strmessage("ERROR: reset IMU\n\n\n");
			DeadReckoner::resetCalibration();
			imuSetup();
		}
		else if (ticksSinceLastImuRead >= 6) {
			bumpImu();
			Serial.printf("IMU bus & fifo reset after %d successful reads\n", totalImuReads);
			totalImuReads = 0;
		}
	}
	else ticksSinceLastImuRead = 0;
	// Do this at 25 Hz
	if (loopCounter % 2 == 1) {
		readAltimeter();
	}

	pilotLoop();
	//Serial.println(micros() - startTime);
	// do at 50 Hz
	airspeedCalc::readAirspeed();
	//Serial.println(micros() - startTime);

	readGps();

	if (startTime > lastPrintTime + 200000) {
		if (tickImuReads > 0) {
			DeadReckoner::printData();
			lastPrintTime = startTime;
			//Serial.printf("ax=%f, ay=%f, az=%f; ", imuData.accelx, imuData.accely, imuData.accelz);
			//Serial.printf("gx=%f, gy=%f, gz=%f\n", imuData.gyrox, imuData.gyroy, imuData.gyroz);
			telem_airspeed(airspeedCalc::airspeed, airspeedCalc::avgPressureDiff);
		}
	}
	int endTime = micros();
	//Serial.printf("loop took %d us\n", endTime - startTime);
	// make loop
	int delayTime = loopInterval - (endTime - startTime);
	if (delayTime < 0) {
		Serial.print("Warning: loop ran over by "); Serial.print(-delayTime); Serial.println(" microseconds");
	}
	else {
		delayMicroseconds(delayTime);
	}
}
