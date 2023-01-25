#include <Arduino.h>
#include "sensorcomm.h"
#include "telemetry.h"
#include "inertial.h"
#include <cmath>
#include "ringbuffer.h"
#include <SparkFunMPU9250-DMP.h>
#include <cstdint>
#include <i2c_driver.h>
#include <i2c_device.h>
#include <imx_rt1060/imx_rt1060_i2c_driver.h>

float baromAltitude;


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

	// Pin for autopilot enabled signal
	pinMode(RELAY_PIN, INPUT_PULLUP);

	// so there isn't a big queue of imu values
	bumpImu();
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

// 1MHz data rate
constexpr int IMU_I2C_CLOCK = 1000000;
void i2cSetup() {
	// Start the first I2C bus (MPU9250)
	//pinMode(18, INPUT_PULLUP);
	//pinMode(19, INPUT_PULLUP);

	Wire.begin();
	Wire.setClock(IMU_I2C_CLOCK);

	// start the second I2C bus (BMP390), Wire1, on pins 16 and 17
	//pinMode(16, INPUT_PULLUP);
	//pinMode(17, INPUT_PULLUP);
	//Wire1.begin();
	//Wire1.setClock(1000000);
	//Wire1.setWireTimeout(2000, false); // This should be quick so barometer resets don't hang stuff

	Master1.begin(1000000);


	//pinMode(24, INPUT_PULLUP);
	//pinMode(25, INPUT_PULLUP);
	//Wire2.begin();
	//Wire2.setClock(1000000);
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
extern "C" {
#include "util/inv_mpu.h"
}

MPU9250_DMP imu;

inv_error_t MPU9250_DMP::dmpSetAccelBias(long * bias) {
	return dmp_set_accel_bias(bias);
}

void imuSetup() {
	//bool wasOn = imu.fifoAvailable();
	// Call imu.begin() to verify communication and initialize
	if (/*!wasOn && */imu.begin() != INV_SUCCESS) {
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

	//if (!wasOn) {
		imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		DMP_FEATURE_6X_LP_QUAT |                                // Enable 6-axis quat
		DMP_FEATURE_GYRO_CAL,                                   // Use gyro calibration
		200);                                                    // Set DMP FIFO rate to 200 Hz
		// DMP_FEATURE_LP_QUAT can also be used. It uses the
		// accelerometer in low-power mode to estimate quat's.
		// DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
	//}

	long bias[3];
	mpu_read_6500_accel_bias(bias);
	Serial.printf("accel bias: 0: %d 1: %d 2: %d\n", bias[0], bias[1], bias[2]); // units: 1/4096 g ?

	//if (!wasOn) {
		// for our HiLetGo MPU9255: Calibrated accelerometer biases: x=-0.015164 y=-0.006503 z=0.101141
		//long newbias[] = {31, 13, -207}; // units: 1/2048 g
		// For our Gy-91 MPU6500: Calibrated accelerometer biases: x=0.006198 y=-0.001784 z=-0.006231
		long newbias[] = {-13, 4, 13}; // units: 1/2048 g
		mpu_set_accel_bias_6500_reg(newbias);

		mpu_read_6500_accel_bias(bias);
		Serial.printf("new accel bias: 0: %d 1: %d 2: %d\n", bias[0], bias[1], bias[2]);

		// gyro bias for Gy-91 MPU6500
		// Startup gyro average: x=1.575864 y=0.444146 z=1.108477
		// not sure if this format is correct
		//long gyrobias[] = { -26, -7, -18 };
		// This doesn't seem to do anything
		//dmp_set_gyro_bias(gyrobias);
	//}

}

bool readImu() {
	// this gets reset to 100kHz somewhere, don't know where, so we have to set it here
	Wire.setClock(IMU_I2C_CLOCK);

	// Check for new data in the FIFO
	if (imu.fifoAvailable()) {
		// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
		inv_error_t result = imu.dmpUpdateFifo();
		if (result == INV_SUCCESS) {
			return true;
		}
		else {
			Serial.printf("Error reading imu fifo: %d\n", result);
			//bumpImu();
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

/*
extern "C" {
int mpu_read_6500_gyro_bias(long *gyro_bias);
}
void printGyroBiases() {
	long bias[3];
	mpu_read_6500_gyro_bias(bias);
	Serial.printf("gyro bias: x: %d y: %d z: %d\n", bias[0], bias[1], bias[2]);
}*/

#include <bmp3_defs.h>
#include <bmp3.h>
#include <bmp2_defs.h>
#include <bmp2.h>

struct bmp3_dev bmpdev;
struct bmp3_settings bmpsettings = { 0 };

// Stuff that uses the Gemmell teensy-specific i2c library
namespace BmpFuncsGemmell {

	uint8_t BMP_I2C_ADDRESS = 0x77;
	I2CMaster* i2c_dev = &Master1;
	constexpr ulong i2c_timeout_us = 2000;

	int bytesLastInCall = 0;
	bool i2c_wait(int bytesLastInCall) {
		unsigned timeout = i2c_timeout_us + bytesLastInCall * 80;
		ulong startTime = micros();
		while (!i2c_dev->finished()) {
			if (micros() - startTime > timeout) return false;
		}
		return true;
	}

	int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                void *intf_ptr) {
		//Serial.print("I2C read address 0x"); Serial.print(reg_addr, HEX);
		//Serial.print(" len "); Serial.println(len, HEX);

		//if (!i2c_dev->write_then_read(&reg_addr, 1, reg_data, len))
		//	return 1;

		i2c_dev->write_async(BMP_I2C_ADDRESS, &reg_addr, 1, false);
		i2c_wait(1);
		if (i2c_dev->get_bytes_transferred() != 1) return BMP3_E_COMM_FAIL;

		//Serial.printf("writed, result=%i\n", result);

		i2c_dev->read_async(BMP_I2C_ADDRESS, reg_data, len, true);
		i2c_wait(len);
		if (i2c_dev->get_bytes_transferred() != len) return BMP3_E_COMM_FAIL;
		//Serial.println("requested");

		return BMP3_OK;
	}

	int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
			void *intf_ptr) {
		// Serial.print("I2C write address 0x"); Serial.print(reg_addr, HEX);
		// Serial.print(" len "); Serial.println(len, HEX);

		//if (!i2c_dev->write((uint8_t *)reg_data, len, true, &reg_addr, 1))
		//	return 1;
		i2c_dev->write_async(BMP_I2C_ADDRESS, &reg_addr, 1, false);
		i2c_wait(1);
		if (i2c_dev->get_bytes_transferred() != 1) return BMP3_E_COMM_FAIL;
		i2c_dev->write_async(BMP_I2C_ADDRESS, const_cast<uint8_t*>(reg_data), len, true);
		i2c_wait(len);
		if (i2c_dev->get_bytes_transferred() != len) return BMP3_E_COMM_FAIL;

		return BMP3_OK;
	}
}

// stuff that uses the Wire library (yes, code duplication, yes, bad)
// Wire library can't do timeout stuff but is needed for compatibility with
// the IMU on the same bus
namespace BmpFuncs {
	uint8_t BMP_I2C_ADDRESS = 0x76;
	TwoWire* i2c_dev;

	int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                void *intf_ptr) {
		//Serial.print("I2C read address 0x"); Serial.print(reg_addr, HEX);
		//Serial.print(" len "); Serial.println(len, HEX);

		//if (!i2c_dev->write_then_read(&reg_addr, 1, reg_data, len))
		//	return 1;

		i2c_dev->beginTransmission(BMP_I2C_ADDRESS);
		unsigned result = i2c_dev->write(&reg_addr, 1);
		if (result != 1) return BMP2_E_COM_FAIL;
		result = i2c_dev->endTransmission(false);
		//Serial.printf("writed, result=%i\n", result);
		if (result != 0) return BMP2_E_COM_FAIL;

		result = i2c_dev->requestFrom(BMP_I2C_ADDRESS, len, true);
		if (result != len) return BMP2_E_COM_FAIL;
		//Serial.println("requested");

		for (unsigned i = 0; i < len; ++i) {
			reg_data[i] = i2c_dev->read();
		}

		return BMP2_OK;
	}

	int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
			void *intf_ptr) {
		// Serial.print("I2C write address 0x"); Serial.print(reg_addr, HEX);
		// Serial.print(" len "); Serial.println(len, HEX);

		//if (!i2c_dev->write((uint8_t *)reg_data, len, true, &reg_addr, 1))
		//	return 1;
		i2c_dev->beginTransmission(BMP_I2C_ADDRESS);
		unsigned result = i2c_dev->write(&reg_addr, 1);
		if (result != 1) return BMP2_E_COM_FAIL;
		result = i2c_dev->write(reg_data, len);
		if (result != len) return BMP2_E_COM_FAIL;
		result = i2c_dev->endTransmission();
		if (result != 0) return BMP2_E_COM_FAIL;

		return BMP2_OK;
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
            bmp3->read = &BmpFuncsGemmell::i2c_read;
            bmp3->write = &BmpFuncsGemmell::i2c_write;
            bmp3->intf = BMP3_I2C_INTF;

			/*
            // SDO pin is made low
            (void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
            (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);*/
        }
        // else SPI; we not doing SPI here

        bmp3->delay_us = &BmpFuncs::delay_usec;
        bmp3->intf_ptr = &BmpFuncsGemmell::BMP_I2C_ADDRESS;
    }
    else {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}

void bmp390Setup() {
	ulong startTime = micros();
	Serial.println("Initializing BMP3XX...");


    int8_t rslt = bmp3_interface_init(&bmpdev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

	ulong ifaceInitDone = micros();

    rslt = bmp3_init(&bmpdev);
    bmp3_check_rslt("bmp3_init", rslt);
	ulong initDone = micros();

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
	ulong settingsDone = micros();

    bmpsettings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&bmpsettings, &bmpdev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);
	ulong modeSet = micros();

	Serial.printf("Barometer setup timing: iface %d, init %d, settings %d, op_mode %d\n", ifaceInitDone - startTime, initDone - ifaceInitDone, settingsDone - initDone, modeSet - settingsDone);
}

int altimeterResets = 0;
int altimeterBadTicks = 0;

void gotPressure(float pascals, float temperature) {
	float atmospheric = pascals / 100.0F;
	baromAltitude = 44330.0 * (1.0 - pow(atmospheric / 1013.25, 0.1903));
	telem_pressureTemp(atmospheric, temperature, baromAltitude);
}

void readBmp390() {
	if (altimeterResets == 3) {
		telem_strmessage("ERROR: bad altimeter\n\n\n");
	}

    struct bmp3_status status = { { 0 } };
	struct bmp3_data data = { 0 };
	int8_t rslt = bmp3_get_status(&status, &bmpdev);
	bmp3_check_rslt("bmp3_get_status", rslt);

	if (rslt != BMP3_OK) {
		// reset barometer (it sometimes dies for some reason???)
		altimeterSetup();
		++altimeterResets;
	}
	/* Read temperature and pressure data iteratively based on data ready interrupt */
	else if (status.intr.drdy == BMP3_ENABLE) {
		/*
			* First parameter indicates the type of data to be read
			* BMP3_PRESS_TEMP : To read pressure and temperature data
			* BMP3_TEMP       : To read only temperature data
			* BMP3_PRESS      : To read only pressure data
			*/
		rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &bmpdev);
		bmp3_check_rslt("bmp3_get_sensor_data", rslt);

		if (rslt != BMP3_OK) {
			++altimeterBadTicks;
			if (altimeterBadTicks > 10) {
				altimeterSetup();
				++altimeterResets;
				altimeterBadTicks = 0;
			}
		}
		else {
			gotPressure(data.pressure, data.temperature);
			//altimeterResets = 0;
			altimeterBadTicks = 0;
		}

		/* NOTE : Read status register again to clear data ready interrupt status */
		rslt = bmp3_get_status(&status, &bmpdev);
		bmp3_check_rslt("bmp3_get_status", rslt);

		//telem_pressureTemp390(data.pressure / 100.0, data.temperature, baromAltitude);

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
		Serial.print("BMP390 not ready\n");
		++altimeterBadTicks;
		if (altimeterBadTicks > 10) {
			altimeterSetup();
			++altimeterResets;
			altimeterBadTicks = 0;
		}

	}
}

// Adapted from BMP2-sensor-API example code
/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmp2_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP2_OK)
    {
        Serial.printf("%s\t", api_name);

        switch (rslt)
        {
            case BMP2_E_NULL_PTR:
                Serial.printf("Error [%d] : Null pointer error.", rslt);
                Serial.printf(
                    "It occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL.\r\n");
                break;
            case BMP2_E_COM_FAIL:
                Serial.printf("Error [%d] : Communication failure error.", rslt);
                Serial.printf(
                    "It occurs due to read/write operation failure and also due to power failure during communication\r\n");
                break;
            case BMP2_E_INVALID_LEN:
                Serial.printf("Error [%d] : Invalid length error.", rslt);
                Serial.printf("Occurs when length of data to be written is zero\n");
                break;
            case BMP2_E_DEV_NOT_FOUND:
                Serial.printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                       rslt);
                break;
            case BMP2_E_UNCOMP_TEMP_RANGE:
                Serial.printf("Error [%d] : Uncompensated temperature data not in valid range error.", rslt);
                break;
            case BMP2_E_UNCOMP_PRESS_RANGE:
                Serial.printf("Error [%d] : Uncompensated pressure data not in valid range error.", rslt);
                break;
            case BMP2_E_UNCOMP_TEMP_AND_PRESS_RANGE:
                Serial.printf(
                    "Error [%d] : Uncompensated pressure data and uncompensated temperature data are not in valid range error.",
                    rslt);
                break;
            default:
                Serial.printf("Error [%d] : Unknown error code\r\n", rslt);
                break;
        }
    }
}

int8_t bmp2_interface_selection(struct bmp2_dev *dev)
{
    int8_t rslt = BMP2_OK;

    if (dev != NULL) {

		BmpFuncs::i2c_dev = &Wire;

		dev->read = BmpFuncs::i2c_read;
		dev->write = BmpFuncs::i2c_write;
		dev->intf = BMP2_I2C_INTF;

        /* Holds the I2C device addr or SPI chip selection */
        dev->intf_ptr = &BmpFuncs::BMP_I2C_ADDRESS;

        /* Configure delay in microseconds */
        dev->delay_us = BmpFuncs::delay_usec;
    }
    else {
        rslt = BMP2_E_NULL_PTR;
    }

    return rslt;
}

struct bmp2_dev bmp2;

void bmp280Setup() {
	int8_t rslt;
    uint32_t meas_time;
    struct bmp2_config conf;

	rslt = bmp2_interface_selection(&bmp2);
    bmp2_error_codes_print_result("bmp2_interface_selection", rslt);

    rslt = bmp2_init(&bmp2);
    bmp2_error_codes_print_result("bmp2_init", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bmp2_get_config(&conf, &bmp2);
    bmp2_error_codes_print_result("bmp2_get_config", rslt);

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    conf.filter = BMP2_FILTER_OFF;

    conf.os_mode = BMP2_OS_MODE_ULTRA_HIGH_RESOLUTION;

    /* Setting the output data rate */
    //conf.odr = BMP2_ODR_0_5_MS;
    conf.odr = BMP2_ODR_250_MS;

    rslt = bmp2_set_config(&conf, &bmp2);
    bmp2_error_codes_print_result("bmp2_set_config", rslt);

    /* Set normal power mode */
    rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &conf, &bmp2);
    bmp2_error_codes_print_result("bmp2_set_power_mode", rslt);

    /* Calculate measurement time in microseconds */
    rslt = bmp2_compute_meas_time(&meas_time, &conf, &bmp2);
    bmp2_error_codes_print_result("bmp2_compute_meas_time", rslt);

	Serial.printf("BMP2 measurement time: %d us\n", meas_time);
}

extern "C" {
	int8_t parse_sensor_data(const uint8_t *reg_data, struct bmp2_uncomp_data *uncomp_data);
}

void readBmp280() {
	int8_t rslt = BMP2_E_NULL_PTR;
    struct bmp2_status status;
    struct bmp2_data comp_data;

	rslt = bmp2_get_status(&status, &bmp2);
	bmp2_error_codes_print_result("bmp2_get_status", rslt);

	if (status.measuring == BMP2_MEAS_DONE) {

		// Inspect uncompensated data?

		uint8_t temp[BMP2_P_T_LEN] = { 0 };
		struct bmp2_uncomp_data uncomp_data = { 0 };
		bmp2_get_regs(BMP2_REG_PRES_MSB, temp, BMP2_P_T_LEN, &bmp2);
		parse_sensor_data(temp, &uncomp_data);

		Serial.printf("uncomp sensor data: p: %d, t: %d\n", uncomp_data.pressure, uncomp_data.temperature);

		/* Read compensated data */
		rslt = bmp2_get_sensor_data(&comp_data, &bmp2);
		bmp2_error_codes_print_result("bmp2_get_sensor_data", rslt);

		if (rslt != BMP2_OK) {
			++altimeterBadTicks;
			if (altimeterBadTicks > 10) {
				altimeterSetup();
				++altimeterResets;
				altimeterBadTicks = 0;
			}
		}
		else {
			gotPressure(comp_data.pressure, comp_data.temperature);
			//altimeterResets = 0;
			altimeterBadTicks = 0;
		}
	}
	else {
		Serial.print("BMP280 not ready\n");
		++altimeterBadTicks;
		if (altimeterBadTicks > 10) {
			altimeterSetup();
			++altimeterResets;
			altimeterBadTicks = 0;
		}
	}
}


void altimeterSetup() {
	//bmp280Setup();
	bmp390Setup();
}
void readAltimeter() {
	//readBmp280();
	readBmp390();
}

float getBaromAltitude(){
	return baromAltitude;
}

namespace airspeedCalc {

	float airspeed = 0;

	constexpr int AIRSPEED_ADDRESS = 0x28;
	constexpr int AIRSPEED_I2C_CLOCK = 1000000;
	I2CMaster* master = &Master2;
	IntervalTimer pollingTimer;

	// data conversion
	constexpr int16_t P_CNT_ = 16383;
    constexpr int16_t T_CNT_ = 2047;
	constexpr float p_max = 1.0, p_min = -1.0;
	// for output type A
	constexpr float kc = 0.1, kd = 0.8;

	// for converting raw pressure values into airspeed
	constexpr float AIR_DENSITY = 1.204; //kg/m^3. Might calculate en suite later.
	constexpr float PRESSURE_DIFF_CORRECTION = 101; // to correct for the apparent 101Pa pressure


	constexpr int PRES_BUF_SIZE = 25;
	ring_buffer<float> pressureBuffer(PRES_BUF_SIZE);
	float avgPressureDiff = 0;


	constexpr int SAMPLES_PER_READ = 80; // Allocate enough space to empty the buffer at 25 Hz (target is 50 Hz)
	constexpr int SAMPLE_SIZE = 4;
	uint8_t sensor_buf[SAMPLE_SIZE * SAMPLES_PER_READ];
	volatile int sampleIdx = 0;

	// call at up to 2kHz to get values from the airspeed sensor
	void pollAirspeed() {
			if (master->finished()) {
				if (sampleIdx < SAMPLES_PER_READ) {
				master->read_async(AIRSPEED_ADDRESS, sensor_buf + sampleIdx * SAMPLE_SIZE, SAMPLE_SIZE, false);
			}
			sampleIdx++;
		}
	}
	void airspeedSetup() {
		master->begin(AIRSPEED_I2C_CLOCK);
		pollingTimer.begin(&pollAirspeed, 500);
	}

	int airspeedMisreadTicks = 0;

	void readAirspeed() {
		// Average all the reads since readAirspeed() was last called
		int actualSamples = 0;
		float totalPresCounts = 0;
		for (int i = 0; i < SAMPLES_PER_READ && i < sampleIdx; ++i) {
			uint8_t *buf = sensor_buf + i * SAMPLE_SIZE;
			uint16_t pres_cnts = static_cast<uint16_t>(buf[0] & 0x3F) << 8 | buf[1];
			//uint16_t temp_cnts = static_cast<uint16_t>(buf[2]) << 3 | (buf[3] & 0xE0) >> 5;
			if (pres_cnts != 0) {
				totalPresCounts += pres_cnts;
				actualSamples++;
			}
		}
		sampleIdx = 0;

		if (actualSamples == 0) {
			Serial.println("airspeed sensor not responding");
			++airspeedMisreadTicks;
			if (airspeedMisreadTicks % 10 == 0) {
				// reset the bus
				// disable the interrupt while we do it
				pollingTimer.end();
				master->end();
				airspeedSetup();
			}
			if (airspeedMisreadTicks == 25) {
				// error message, once
				telem_strmessage("ERROR: airspeed out\n\n");
			}
			return;
		}
		airspeedMisreadTicks = 0;

		float avgCnts = totalPresCounts / actualSamples;
		float pres_psi = (avgCnts - kc * P_CNT_) *
              ((p_max - p_min) / (kd * P_CNT_)) + p_min;
		float pres_pa =  pres_psi * 0.45359237f * 9.80665f / 0.0254f / 0.0254f;
		//Serial.printf("pres_psi: %f\n", pres_pa);

		float pressureDiff = pres_pa + PRESSURE_DIFF_CORRECTION; //calculate raw airspeed from pressure differential
		avgPressureDiff += (1.0 / PRES_BUF_SIZE) * pressureDiff;
		if(pressureBuffer.full()) {
			avgPressureDiff -= (1.0 / PRES_BUF_SIZE)*pressureBuffer.pop();
		}
		pressureBuffer.put(pressureDiff); // use a ring buffer to maintain rolling half-second pressure differential average


		airspeed = sqrt(2*fabs(avgPressureDiff)/AIR_DENSITY);
		//telem_airspeed(airspeed, avgPressureDiff);
	}
}
