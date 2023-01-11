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


void loop() {
	++loopCounter;
	//telem_fakedata(123, 4567);
	/*telem_morefakestuff(3.2, 1000000, 2.1673453459345);
	telem_strmessage("blahblah");
	//Serial.print("garbage");
	delay(1000);*/
	//printImuData();


	int startTime = micros();


	readImu();
	RawImuData imuData = getImuData();
	DeadReckoner::newData(imuData);

	if (startTime > lastPrintTime + 200000) {
		DeadReckoner::printData();
		lastPrintTime = startTime;
		/*Serial.println("ax=" +  String(imuData.accelx) + " ay=" + String(imuData.accely) + " az=" + String(imuData.accelz));
		Serial.println("gx=" +  String(imuData.gyrox) + " gy=" + String(imuData.gyroy) + " gz=" + String(imuData.gyroz));*/
	}
	// Do this at 25 Hz
	if (loopCounter % 8 == 0) {
		readAltimeter();
    readAirspeed();
	}

	int endTime = micros();
	// make loop
	int delayTime = loopInterval - (endTime - startTime);
	if (delayTime < 0) {
		Serial.print("Warning: loop ran over by ");
		Serial.print(-delayTime);
		Serial.println(" microseconds");
	}
	else delayMicroseconds(delayTime);
}
