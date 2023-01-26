

#include "telemetry.h"
#include "sensorcomm.h"
#include "inertial.h"
#include "pilot.h"
#include "gpsnav.h"

void setup() {
	setupAllComms();
	GPSNav::gpsSetup();
	setupSdCardTelem();
	pilotSetup();
	telem_strmessage("Initialization complete");
}

// 200Hz
constexpr int loopInterval = 20000; // microseconds

int lastPrintTime = 0;
int loopCounter = 0;

int totalImuReads = 0;
int ticksSinceLastImuRead = 0;


bool autopilotEnabled = false;

void loop() {
	++loopCounter;

	int startTime = micros();


	// Check whether autopilot is enabled
	bool newAutopilotEnabled = !digitalRead(RELAY_PIN);
	if (newAutopilotEnabled && !autopilotEnabled) {
		pilotStart();
		telem_strmessage("AUTOPILOT ENABLED");
	}
	else if (!newAutopilotEnabled && autopilotEnabled) {
		telem_strmessage("AUTOPILOT DISABLED");
	}
	autopilotEnabled = newAutopilotEnabled;


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
			ticksSinceLastImuRead = 0;
		}
		else if (ticksSinceLastImuRead >= 6) {
			bumpImu();
			Serial.printf("IMU bus & fifo reset after %d successful reads\n", totalImuReads);
			totalImuReads = 0;
		}
	}
	else {
		if (ticksSinceLastImuRead != 0) {
			telem_warningImuStalledByTicks(ticksSinceLastImuRead);
		}
		ticksSinceLastImuRead = 0;
	}
	// Do this at 25 Hz
	/*if (loopCounter % 2 == 1) {
		readAltimeter();
	}*/

	//airspeedCalc::readAirspeed();

	pilotLoop();
	//Serial.println(micros() - startTime);
	// do at 50 Hz
	//Serial.println(micros() - startTime);

	GPSNav::updatenav();

	// do at 1 Hz
	if (loopCounter % 50 == 0) {
		// Not ideal (waits for the write to complete), but needs to be done, or else there is sometimes no writing at all
		flushSdCardTelem();
	}

	//if (startTime > lastPrintTime + 200000) {
		if (tickImuReads > 0) {
			DeadReckoner::printData();
			lastPrintTime = startTime;
			//Serial.printf("ax=%f, ay=%f, az=%f; ", imuData.accelx, imuData.accely, imuData.accelz);
			//Serial.printf("gx=%f, gy=%f, gz=%f\n", imuData.gyrox, imuData.gyroy, imuData.gyroz);
			telem_airspeed(airspeedCalc::airspeed, airspeedCalc::avgPressureDiff);
		}
	//}
	int endTime = micros();
	//Serial.printf("loop took %d us\n", endTime - startTime);
	// make loop
	int delayTime = loopInterval - (endTime - startTime);
	if (delayTime < 0) {
		//Serial.print("Warning: loop ran over by "); Serial.print(-delayTime); Serial.println(" microseconds");
		telem_warningLoopRanOverByMicroseconds(-delayTime);
	}
	else {
		delayMicroseconds(delayTime);
	}
}
