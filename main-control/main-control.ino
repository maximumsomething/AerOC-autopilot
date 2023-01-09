#include "telemetry.h"
#include "sensorcomm.h"
#include "inertial.h"

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
		/*Serial.println("ax=" +  String(data.accelx) + " ay=" + String(data.accely) + " az=" + String(data.accelz));
		Serial.println("gx=" +  String(data.gyrox) + " gy=" + String(data.gyroy) + " gz=" + String(data.gyroz));*/
	}
	// Do this at low Hz
	if (loopCounter % (200 / 25) == 0) {
		readAltimeter();
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
