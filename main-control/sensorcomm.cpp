#include "sensorcomm.h"
#include "telemetry.h"
#include "inertial.h"
#include <SparkFunMPU9250-DMP.h>


void setupAllComms() {
	usbSerialSetup();
	telemSerialSetup();
	i2cSetup();
	// i2cScan();
	imuSetup();
}

void usbSerialSetup() {
	Serial.begin(115200);
	while (!Serial) {
		delay(5);
	}
}

void telemSerialSetup() {
	Serial1.begin(9600);
	while (!Serial1) {
		delay(5);
	}
}

void i2cSetup() {
	// Start the I2C bus
	pinMode(18, INPUT_PULLUP);
	pinMode(19, INPUT_PULLUP);

	Wire.begin();
	// lowest data rate, due to internal pullup resistors being bad
	Wire.setClock(100000);
}

void i2cScan() {
	byte error, address;
	int nDevices;

	Serial.println("Scanning I2C...");

	nDevices = 0;
	for (address = 1; address < 127; address++) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		} else if (error == 4) {
			Serial.print("Unknown error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
}

#include <SparkFunMPU9250-DMP.h>

MPU9250_DMP imu;

void imuSetup() {
	// Call imu.begin() to verify communication and initialize
	if (imu.begin() != INV_SUCCESS) {
		while (1) {
			Serial.println("Unable to communicate with MPU-9250");
			Serial.println("Check connections, and try again.");
			Serial.println();
			delay(1000);
		}
	}

	imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);               // Enable gyroscope and accel
	imu.setGyroFSR(2000);                                       // Set gyro to 2000 dps

	imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	DMP_FEATURE_6X_LP_QUAT |                                // Enable 6-axis quat
	DMP_FEATURE_GYRO_CAL,                                   // Use gyro calibration
	200);                                                    // Set DMP FIFO rate to 200 Hz
	// DMP_FEATURE_LP_QUAT can also be used. It uses the
	// accelerometer in low-power mode to estimate quat's.
	// DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void printIMUData() {
	// After calling dmpUpdateFifo() the ax, gx, mx, etc. values
	// are all updated.
	// Quaternion values are, by default, stored in Q30 long
	// format. calcQuat turns them into a float between -1 and 1
	float q0 = imu.calcQuat(imu.qw);
	float q1 = imu.calcQuat(imu.qx);
	float q2 = imu.calcQuat(imu.qy);
	float q3 = imu.calcQuat(imu.qz);

	float ax = imu.calcAccel(imu.ax), ay = imu.calcAccel(imu.ay), az = imu.calcAccel(imu.az);
	float amag = sqrt(ax*ax + ay*ay + az*az);

	Serial.println("Q: " + String(q0, 4) + ", " + String(q1, 4) + ", " +
	String(q2, 4) + ", " + String(q3, 4));
	Serial.println("R/P/Y: " + String(imu.roll) + ", " + String(imu.pitch) +
	", " + String(imu.yaw));
	Serial.println("Accel xyz: " + String(ax) + ", " + String(ay) + ", " + String(az) + " Total: " + String(amag));
	Serial.println("Time: " + String(imu.time) + " ms");
	Serial.println();
}

void readImu() {
	// Check for new data in the FIFO
	if (imu.fifoAvailable()) {
		// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
		if (imu.dmpUpdateFifo() == INV_SUCCESS) {
			imu.computeEulerAngles();
			//printIMUData();
		}
	}
}

RawImuData getImuData() {
	return {
		imu.calcAccel(imu.ax), imu.calcAccel(imu.ay),  imu.calcAccel(imu.az),
		imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
		imu.calcQuat(imu.qw), imu.calcQuat(imu.qx), imu.calcQuat(imu.qy), imu.calcQuat(imu.qz)
	};
}

/*
 # include <Adafruit_BMP280.h>                                          *

 Adafruit_BMP280 bmp; // I2C

 void setup() {
 wiresetup();
 unsigned status = bmp.begin();
 if (!status) {
	 Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
	 "try a different address!"));
	 Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
	 Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or
	 BMP 085\n"); Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
	 Serial.print("        ID of 0x60 represents a BME 280.\n");
	 Serial.print("        ID of 0x61 represents a BME 680.\n");
	 while (1) delay(10);
	 }
	 }

	 void loop() {

	 }
	 */
