#include <Arduino.h>
#include "sensorcomm.h"
#include "telemetry.h"
#include "inertial.h"
#include "ms4525do.h"
#include "cmath"
#include "ringbuffer.h"
#include <SparkFunMPU9250-DMP.h>

bfs::Ms4525do pres;

void setupAllComms() {
	usbSerialSetup();
	telemSerialSetup();
	i2cSetup();
	//i2cScan();
	imuSetup();
	altimeterSetup();
	airspeedCalc::airspeedSetup();
	// status LED
	pinMode(13, OUTPUT);
}

void usbSerialSetup() {
	Serial.begin(115200);
	/*while (!Serial) {
		delay(5);
	}*/
}

void telemSerialSetup() {
	Serial1.begin(9600);
	while (!Serial1) {
		delay(5);
	}
}

void i2cSetup() {
	// Start the first I2C bus (MPU9250)
	//pinMode(18, INPUT_PULLUP);
	//pinMode(19, INPUT_PULLUP);

	Wire.begin();
	// 1MHz data rate
	Wire.setClock(400000);

	// start the second I2C bus (BMP390), Wire1, on pins 16 and 17
	//pinMode(16, INPUT_PULLUP);
	//pinMode(17, INPUT_PULLUP);
	Wire1.begin();
	Wire1.setClock(1000000);

	//pinMode(24, INPUT_PULLUP);
	//pinMode(25, INPUT_PULLUP);
	Wire2.begin();
	Wire2.setClock(1000000);
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
		Wire2.beginTransmission(address);
		error = Wire2.endTransmission();

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
extern "C" {
#include "util/inv_mpu.h"
}

MPU9250_DMP imu;

inv_error_t MPU9250_DMP::dmpSetAccelBias(long * bias) {
	return dmp_set_accel_bias(bias);
}

int g_to_q16(float g) {

}

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

	imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);               // Enable gyroscope and accel
	imu.setGyroFSR(2000);                                       // Set gyro to 2000 dps
	imu.setAccelFSR(8);


	imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	DMP_FEATURE_6X_LP_QUAT |                                // Enable 6-axis quat
	DMP_FEATURE_GYRO_CAL,                                   // Use gyro calibration
	200);                                                    // Set DMP FIFO rate to 200 Hz
	// DMP_FEATURE_LP_QUAT can also be used. It uses the
	// accelerometer in low-power mode to estimate quat's.
	// DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

	long bias[3];
	mpu_read_6500_accel_bias(bias);
	Serial.printf("accel bias: 0: %d 1: %d 2: %d\n", bias[0], bias[1], bias[2]); // units: 1/4096 g ?

	// for our HiLetGo MPU9255: Calibrated accelerometer biases: x=-0.015164 y=-0.006503 z=0.101141
	//long newbias[] = {31, 13, -207}; // units: 1/2048 g
	// For our Gy-91 MPU6500: Calibrated accelerometer biases: x=0.006198 y=-0.001784 z=-0.006231
	long newbias[] = {-13, 4, 13}; // units: 1/2048 g
	mpu_set_accel_bias_6500_reg(newbias);

	mpu_read_6500_accel_bias(bias);
	Serial.printf("new accel bias: 0: %d 1: %d 2: %d\n", bias[0], bias[1], bias[2]);

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

bool readImu() {
	// Check for new data in the FIFO
	if (imu.fifoAvailable()) {
		// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
		inv_error_t result = imu.dmpUpdateFifo();
		if (result == INV_SUCCESS) {
			return true;
		}
		else {
			Serial.printf("Error reading imu fifo: %d\n", result);
			imu.resetFifo();
		}
	}
	return false;
}

void bumpImu() {
	Wire.end();
	Wire.begin();
	imu.resetFifo();
}

RawImuData getImuData() {
	/*imu.update(UPDATE_COMPASS);
	Serial.printf("magX: %f\n", imu.calcMag(imu.mx));*/
	return {
		imu.calcAccel(imu.ax), imu.calcAccel(imu.ay),  imu.calcAccel(imu.az),
		imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
		imu.calcQuat(imu.qw), imu.calcQuat(imu.qx), imu.calcQuat(imu.qy), imu.calcQuat(imu.qz)
	};
}

#include <bmp3_defs.h>
#include <bmp3.h>

struct bmp3_dev bmpdev;
struct bmp3_settings bmpsettings = { 0 };

namespace BmpFuncs {

	TwoWire* i2c_dev;
	uint8_t i2c_address;

	int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                void *intf_ptr) {
		//Serial.print("I2C read address 0x"); Serial.print(reg_addr, HEX);
		//Serial.print(" len "); Serial.println(len, HEX);

		//if (!i2c_dev->write_then_read(&reg_addr, 1, reg_data, len))
		//	return 1;

		i2c_dev->beginTransmission(i2c_address);
		unsigned result = i2c_dev->write(&reg_addr, 1);
		if (result != 1) return BMP3_E_COMM_FAIL;
		result = i2c_dev->endTransmission(false);
		//Serial.printf("writed, result=%i\n", result);
		if (result != 0) return BMP3_E_COMM_FAIL;

		result = i2c_dev->requestFrom(i2c_address, len, true);
		if (result != len) return BMP3_E_COMM_FAIL;
		//Serial.println("requested");

		for (unsigned i = 0; i < len; ++i) {
			reg_data[i] = i2c_dev->read();
		}

		return BMP3_OK;
	}

	int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
			void *intf_ptr) {
		// Serial.print("I2C write address 0x"); Serial.print(reg_addr, HEX);
		// Serial.print(" len "); Serial.println(len, HEX);

		//if (!i2c_dev->write((uint8_t *)reg_data, len, true, &reg_addr, 1))
		//	return 1;
		i2c_dev->beginTransmission(i2c_address);
		unsigned result = i2c_dev->write(&reg_addr, 1);
		if (result != 1) return BMP3_E_COMM_FAIL;
		result = i2c_dev->write(reg_data, len);
		if (result != len) return BMP3_E_COMM_FAIL;
		result = i2c_dev->endTransmission();
		if (result != 0) return BMP3_E_COMM_FAIL;

		return BMP3_OK;
	}

	void delay_usec(uint32_t us, void *intf_ptr) { delayMicroseconds(us); }
}

// code adapted from BMP3-Sensor-API example code

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            Serial.printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            Serial.printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            Serial.printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            Serial.printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            Serial.printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            Serial.printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            Serial.printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            Serial.printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf) {
    int8_t rslt = BMP3_OK;

    if (bmp3 != NULL) {

        /* Bus configuration : I2C */
        if (intf == BMP3_I2C_INTF) {
            Serial.printf("I2C Interface\n");
            //BmpFuncs::i2c_address = BMP3_ADDR_I2C_PRIM;
			BmpFuncs::i2c_address = 0x77;
            bmp3->read = &BmpFuncs::i2c_read;
            bmp3->write = &BmpFuncs::i2c_write;
            bmp3->intf = BMP3_I2C_INTF;

			/*
            // SDO pin is made low
            (void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
            (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);*/
        }
        // else SPI; we not doing SPI here

        bmp3->delay_us = &BmpFuncs::delay_usec;
        bmp3->intf_ptr = &BmpFuncs::i2c_address;
    }
    else {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}

void altimeterSetup() {
	Serial.println("Initializing BMP3XX...");

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
	BmpFuncs::i2c_dev = &Wire1;

    int8_t rslt = bmp3_interface_init(&bmpdev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&bmpdev);
    bmp3_check_rslt("bmp3_init", rslt);

	Serial.println("init device");

    bmpsettings.int_settings.drdy_en = BMP3_ENABLE;
    bmpsettings.press_en = BMP3_ENABLE;
    bmpsettings.temp_en = BMP3_ENABLE;

    bmpsettings.odr_filter.press_os = BMP3_OVERSAMPLING_16X;
    bmpsettings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    bmpsettings.odr_filter.odr = BMP3_ODR_25_HZ;
	bmpsettings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_7;

    uint16_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                   BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &bmpsettings, &bmpdev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    bmpsettings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&bmpsettings, &bmpdev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

	Serial.println("Barometer set up");
}

void readAltimeter() {
    struct bmp3_status status = { { 0 } };
	struct bmp3_data data = { 0 };
	int8_t rslt = bmp3_get_status(&status, &bmpdev);
	bmp3_check_rslt("bmp3_get_status", rslt);

	if (rslt != BMP3_OK) {
		// reset barometer (it sometimes dies for some reason???)
		altimeterSetup();
	}
	/* Read temperature and pressure data iteratively based on data ready interrupt */
	else if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
		/*
			* First parameter indicates the type of data to be read
			* BMP3_PRESS_TEMP : To read pressure and temperature data
			* BMP3_TEMP       : To read only temperature data
			* BMP3_PRESS      : To read only pressure data
			*/
		rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &bmpdev);
		bmp3_check_rslt("bmp3_get_sensor_data", rslt);

		/* NOTE : Read status register again to clear data ready interrupt status */
		rslt = bmp3_get_status(&status, &bmpdev);
		bmp3_check_rslt("bmp3_get_status", rslt);

		float atmospheric = data.pressure / 100.0F;
		float altitude = 44330.0 * (1.0 - pow(atmospheric / 1013.25, 0.1903));

		telem_pressureTemp390(data.pressure / 100.0, data.temperature, altitude);

		/*
		#ifdef BMP3_FLOAT_COMPENSATION
		printf("Data[%d]  T: %.2f deg C, P: %.2f Pa\n", loop, (data.temperature), (data.pressure));
		#else
		printf("Data[%d]  T: %ld deg C, P: %lu Pa\n", loop, (long int)(int32_t)(data.temperature / 100),
				(long unsigned int)(uint32_t)(data.pressure / 100));
		#endif

		loop = loop + 1;*/
	}
	else {
		Serial.print("Barometer not ready\n");
	}
}

namespace airspeedCalc {
	float airspeed = 0;

	ring_buffer<float> pressureBuffer(100, 0);
	float avgPressureDiff = 0;

	void airspeedSetup(){
		pres.Config(&Wire2, 0x28, 1.0f, -1.0f);
		if (!pres.Begin()) {
			Serial.println("Error communicating with barometric altimiter");
		}
	}

	void readAirspeed(){
		const float AIR_DENSITY = 1.204; //kg/m^3. Might calculate en suite later.
		const float PRESSURE_DIFF_CORRECTION = 95.5; // to correct for the apparent 91Pa pressure differential that the sensor seems to output at rest

		if(pres.Read()){	
			float pressureDiff = pres.pres_pa() + PRESSURE_DIFF_CORRECTION; //calculate raw airspeed from pressure differential
			pressureBuffer.put(pressureDiff); // use a ring buffer to maintain rolling half-second pressure differential average
			avgPressureDiff += .01 * pressureDiff;
			if(pressureBuffer.full()){
				avgPressureDiff -= .01*pressureBuffer.pop();
			}
			
			airspeed = sqrt(2*fabs(avgPressureDiff)/AIR_DENSITY);
			//telem_airspeed(airspeed, avgPressureDiff);
		}else{
			Serial.print("Error communicating with airspeed sensor\n");
		}

	}
}
