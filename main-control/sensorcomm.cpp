#include "sensorcomm.h"
#include "telemetry.h"
//#include "mpu9250.h"
//#include "mpu6500.h"
#include "MPU9250.h"
#include "inertial.h"
#include "quaternionFilters.h"

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

	imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL |
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
			printIMUData();
		}
	}
}


#ifdef MPU_WINER

// Modified from library example code.
# define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
# define I2Cclock 100000
# define I2Cport Wire
constexpr bool SerialDebug = true;
constexpr bool AHRS = true;
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void imuSetup() {

	// Read the WHO_AM_I register, this is a good test of communication
	byte whoami = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	Serial.print(F("MPU9250 I AM 0x"));
	Serial.print(whoami, HEX);
	Serial.print(F(" I should be 0x"));
	Serial.println(0x71, HEX);

	// Start by performing self test and reporting values
	myIMU.MPU9250SelfTest(myIMU.selfTest);
	Serial.print(F("x-axis self test: acceleration trim within : "));
	Serial.print(myIMU.selfTest[0], 1);
	Serial.println("% of factory value");
	Serial.print(F("y-axis self test: acceleration trim within : "));
	Serial.print(myIMU.selfTest[1], 1);
	Serial.println("% of factory value");
	Serial.print(F("z-axis self test: acceleration trim within : "));
	Serial.print(myIMU.selfTest[2], 1);
	Serial.println("% of factory value");
	Serial.print(F("x-axis self test: gyration trim within : "));
	Serial.print(myIMU.selfTest[3], 1);
	Serial.println("% of factory value");
	Serial.print(F("y-axis self test: gyration trim within : "));
	Serial.print(myIMU.selfTest[4], 1);
	Serial.println("% of factory value");
	Serial.print(F("z-axis self test: gyration trim within : "));
	Serial.print(myIMU.selfTest[5], 1);
	Serial.println("% of factory value");

	// Calibrate gyro and accelerometers, load biases in bias registers
	myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

	myIMU.initMPU9250();
	// Initialize device for active mode read of acclerometer, gyroscope, and
	// temperature
	Serial.println("MPU9250 initialized for active data mode....");

	// Get sensor resolutions, only need to do this once
	myIMU.getAres();
	myIMU.getGres();
	myIMU.getMres();
}

void magnetometerSetup() {
	// Read the WHO_AM_I register of the magnetometer, this is a good test of
	// communication
	byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
	Serial.print("AK8963 ");
	Serial.print("I AM 0x");
	Serial.print(d, HEX);
	Serial.print(" I should be 0x");
	Serial.println(0x48, HEX);

	if (d != 0x48) {
		// Communication failed, stop here
		Serial.println(F("Communication failed, abort!"));
		Serial.flush();
		abort();
	}
	// Get magnetometer calibration from AK8963 ROM
	myIMU.initAK8963(myIMU.factoryMagCalibration);
	// Initialize device for active mode read of magnetometer
	Serial.println("AK8963 initialized for active data mode....");

	//  Serial.println("Calibration values: ");
	Serial.print("X-Axis factory sensitivity adjustment value ");
	Serial.println(myIMU.factoryMagCalibration[0], 2);
	Serial.print("Y-Axis factory sensitivity adjustment value ");
	Serial.println(myIMU.factoryMagCalibration[1], 2);
	Serial.print("Z-Axis factory sensitivity adjustment value ");
	Serial.println(myIMU.factoryMagCalibration[2], 2);

	//    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
	Serial.println("AK8963 mag biases (mG)");
	Serial.println(myIMU.magBias[0]);
	Serial.println(myIMU.magBias[1]);
	Serial.println(myIMU.magBias[2]);

	Serial.println("AK8963 mag scale (mG)");
	Serial.println(myIMU.magScale[0]);
	Serial.println(myIMU.magScale[1]);
	Serial.println(myIMU.magScale[2]);
	//    delay(2000); // Add delay to see results before serial spew of data

	if (SerialDebug) {
		Serial.println("Magnetometer:");
		Serial.print("X-Axis sensitivity adjustment value ");
		Serial.println(myIMU.factoryMagCalibration[0], 2);
		Serial.print("Y-Axis sensitivity adjustment value ");
		Serial.println(myIMU.factoryMagCalibration[1], 2);
		Serial.print("Z-Axis sensitivity adjustment value ");
		Serial.println(myIMU.factoryMagCalibration[2], 2);
	}
}

void readImu() {
	// On interrupt, check if data ready interrupt
	if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
		myIMU.readAccelData(myIMU.accelCount);                     // Read the x/y/z adc values

		// Now we'll calculate the accleration value into actual g's
		// This depends on scale being set
		myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;        // - myIMU.accelBias[0];
		myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;        // - myIMU.accelBias[1];
		myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;        // - myIMU.accelBias[2];

		myIMU.readGyroData(myIMU.gyroCount);                       // Read the x/y/z adc values

		// Calculate the gyro value into actual degrees per second
		// This depends on scale being set
		myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
		myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
		myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

		if (false) {
			myIMU.readMagData(myIMU.magCount);                         // Read the x/y/z adc values

			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental
			// corrections
			// Get actual magnetometer value, this depends on scale being set
			myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes *
			myIMU.factoryMagCalibration[0] -
			myIMU.magBias[0];
			myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes *
			myIMU.factoryMagCalibration[1] -
			myIMU.magBias[1];
			myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes *
			myIMU.factoryMagCalibration[2] -
			myIMU.magBias[2];
		}
	}                                                           // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

	// Must be called before updating quaternions!
	myIMU.updateTime();

	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
	// the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
	// (+ up) of accelerometer and gyro! We have to make some allowance for this
	// orientationmismatch in feeding the output to the quaternion filter. For the
	// MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
	// along the x-axis just like in the LSM9DS0 sensor. This rotation can be
	// modified to allow any convenient orientation convention. This is ok by
	// aircraft orientation standards! Pass gyro rate as rad/s
	MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
						   myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
						myIMU.mx, myIMU.mz, myIMU.deltat);

	if (!AHRS) {
		myIMU.delt_t = millis() - myIMU.count;
		if (myIMU.delt_t > 500) {
			if (SerialDebug) {
				// Print acceleration values in milligs!
				Serial.print("X-acceleration: ");
				Serial.print(1000 * myIMU.ax);
				Serial.print(" mg ");
				Serial.print("Y-acceleration: ");
				Serial.print(1000 * myIMU.ay);
				Serial.print(" mg ");
				Serial.print("Z-acceleration: ");
				Serial.print(1000 * myIMU.az);
				Serial.println(" mg ");

				// Print gyro values in degree/sec
				Serial.print("X-gyro rate: ");
				Serial.print(myIMU.gx, 3);
				Serial.print(" degrees/sec ");
				Serial.print("Y-gyro rate: ");
				Serial.print(myIMU.gy, 3);
				Serial.print(" degrees/sec ");
				Serial.print("Z-gyro rate: ");
				Serial.print(myIMU.gz, 3);
				Serial.println(" degrees/sec");

				// Print mag values in degree/sec
				Serial.print("X-mag field: ");
				Serial.print(myIMU.mx);
				Serial.print(" mG ");
				Serial.print("Y-mag field: ");
				Serial.print(myIMU.my);
				Serial.print(" mG ");
				Serial.print("Z-mag field: ");
				Serial.print(myIMU.mz);
				Serial.println(" mG");

				myIMU.tempCount = myIMU.readTempData();                   // Read the adc values
				// Temperature in degrees Centigrade
				myIMU.temperature = ((float)myIMU.tempCount) / 333.87 + 21.0;
				// Print temperature in degrees Centigrade
				Serial.print("Temperature is ");
				Serial.print(myIMU.temperature, 1);
				Serial.println(" degrees C");
			}

			myIMU.count = millis();
		}                                                          // if (myIMU.delt_t > 500)
	}                                                           // if (!AHRS)
	else {
		// Serial print and/or display at 0.5 s rate independent of data rates
		myIMU.delt_t = millis() - myIMU.count;

		// update LCD once per half-second independent of read rate
		if (myIMU.delt_t > 500) {
			if (SerialDebug) {
				Serial.print("ax = ");
				Serial.print((int)1000 * myIMU.ax);
				Serial.print(" ay = ");
				Serial.print((int)1000 * myIMU.ay);
				Serial.print(" az = ");
				Serial.print((int)1000 * myIMU.az);
				Serial.println(" mg");

				Serial.print("gx = ");
				Serial.print(myIMU.gx, 2);
				Serial.print(" gy = ");
				Serial.print(myIMU.gy, 2);
				Serial.print(" gz = ");
				Serial.print(myIMU.gz, 2);
				Serial.println(" deg/s");

				Serial.print("mx = ");
				Serial.print((int)myIMU.mx);
				Serial.print(" my = ");
				Serial.print((int)myIMU.my);
				Serial.print(" mz = ");
				Serial.print((int)myIMU.mz);
				Serial.println(" mG");

				Serial.print("q0 = ");
				Serial.print(*getQ());
				Serial.print(" qx = ");
				Serial.print(*(getQ() + 1));
				Serial.print(" qy = ");
				Serial.print(*(getQ() + 2));
				Serial.print(" qz = ");
				Serial.println(*(getQ() + 3));
			}

			// Define output variables from updated quaternion---these are Tait-Bryan
			// angles, commonly used in aircraft orientation. In this coordinate
			// system, the positive z-axis is down toward Earth. Yaw is the angle
			// between Sensor x-axis and Earth magnetic North (or true North if
			// corrected for local declination, looking down on the sensor positive
			// yaw is counterclockwise. Pitch is angle between sensor x-axis and Earth
			// ground plane, toward the Earth is positive, up toward the sky is
			// negative. Roll is angle between sensor y-axis and Earth ground plane,
			// y-axis up is positive roll. These arise from the definition of the
			// homogeneous rotation matrix constructed from quaternions. Tait-Bryan
			// angles as well as Euler angles are non-commutative; that is, the get
			// the correct orientation the rotations must be applied in the correct
			// order which for this configuration is yaw, pitch, and then roll. For
			// more see
			// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
			// which has additional links.
			myIMU.yaw = atan2(
				2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)),
							  *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) -
							  *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
			myIMU.pitch = -asin(
				2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
			myIMU.roll = atan2(
				2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)),
							   *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) -
							   *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
			myIMU.pitch *= RAD_TO_DEG;
			myIMU.yaw *= RAD_TO_DEG;

			// Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
			//    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
			// - http://www.ngdc.noaa.gov/geomag-web/#declination
			myIMU.yaw -= 8.5;
			myIMU.roll *= RAD_TO_DEG;

			if (SerialDebug) {
				Serial.print("Yaw, Pitch, Roll: ");
				Serial.print(myIMU.yaw, 2);
				Serial.print(", ");
				Serial.print(myIMU.pitch, 2);
				Serial.print(", ");
				Serial.println(myIMU.roll, 2);

				Serial.print("rate = ");
				Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
				Serial.println(" Hz");
			}

			// With these settings the filter is updating at a ~145 Hz rate using the
			// Madgwick scheme and >200 Hz using the Mahony scheme even though the
			// display refreshes at only 2 Hz. The filter update rate is determined
			// mostly by the mathematical steps in the respective algorithms, the
			// processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer
			// ODR: an ODR of 10 Hz for the magnetometer produce the above rates,
			// maximum magnetometer ODR of 100 Hz produces filter update rates of 36 -
			// 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. This
			// is presumably because the magnetometer read takes longer than the gyro
			// or accelerometer reads. This filter update rate should be fast enough
			// to maintain accurate platform orientation for stabilization control of
			// a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
			// produced by the on-board Digital Motion Processor of Invensense's
			// MPU6050 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is
			// doing pretty well!

			myIMU.count = millis();
			myIMU.sumCount = 0;
			myIMU.sum = 0;
		}                                                          // if (myIMU.delt_t > 500)
	}                                                           // if (AHRS)
}

#endif                                                      // MPU_WINER

/*
 / / Mpu9250 object                                                     *
 //bfs::Mpu9250 imu;
 bfs::Mpu6500 imu;
 // Polling interrupt
 IntervalTimer imuPoller;

 void imuPoll() {
 RawImuData data = readImu();
 DeadReckoner::newData(data);
 }

 void imuSetup() {

 // I2C bus,  0x68 address
 //imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
 imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
 // Initialize and configure IMU
 while (!imu.Begin()) {

	 Serial.println("Error initializing communication with IMU");
	 delay(1000);
	 }
	 // Set the sample rate divider
	 // srd of 4 means 200Hz sample rate
	 if (!imu.ConfigSrd(4)) {
		 Serial.println("Error configured SRD");
		 while(1) { delay(1000); }
		 }

		 // Set up a interrupt to poll the IMU every 10 milliseconds
		 //imuPoller.begin(&imuPoll, 10000);
		 }




		 RawImuData readImu() {
		 if (imu.Read() && imu.new_imu_data()) {

			 }
			 return { imu.accel_x_mps2(), imu.accel_y_mps2(), imu.accel_z_mps2(),
			 imu.gyro_x_radps(), imu.gyro_y_radps(), imu.gyro_z_radps() };
			 }


			 void printImuData() {
			 // Check if data read
			 //if (imu.Read()) {
			 Serial.print(imu.new_imu_data());
			 Serial.print("\t");
			 //Serial.print(imu.new_mag_data());
			 //Serial.print("\t");
			 Serial.print(imu.accel_x_mps2());
			 Serial.print("\t");
			 Serial.print(imu.accel_y_mps2());
			 Serial.print("\t");
			 Serial.print(imu.accel_z_mps2());
			 Serial.print("\t");
			 Serial.print(imu.gyro_x_radps());
			 Serial.print("\t");
			 Serial.print(imu.gyro_y_radps());
			 Serial.print("\t");
			 Serial.print(imu.gyro_z_radps());
			 Serial.print("\t");
			 Serial.print(imu.mag_x_ut());
			 Serial.print("\t");
			 Serial.print(imu.mag_y_ut());
			 Serial.print("\t");
			 Serial.print(imu.mag_z_ut());
			 Serial.print("\t");
			 Serial.print(imu.die_temp_c());
			 Serial.print("\n");
			 //}
			 }*/
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
/*
 # include <MPU9250_asukiaaa.h>                                         *
 MPU9250_asukiaaa mySensor;
 float aX, aY, aZ, aSqrt;

 void setup() {

 Serial.begin(115200);
 while(!Serial) { delay(5); }

 Wire.begin();
 mySensor.setWire(&Wire);
 mySensor.beginAccel();
 }

 void loop() {
 uint8_t sensorId;
 int result = mySensor.readId(&sensorId);
 if (result == 0) {
	 Serial.println("sensorId: " + String(sensorId));
	 } else {
		 Serial.println("Cannot read sensorId " + String(result));
		 }

		 mySensor.accelUpdate();
		 aX = mySensor.accelX();
		 aY = mySensor.accelY();
		 aZ = mySensor.accelZ();
		 aSqrt = mySensor.accelSqrt();

		 Serial.print(aX); Serial.print("\t");
		 Serial.print("\n");

		 delay(100);
		 }*/
