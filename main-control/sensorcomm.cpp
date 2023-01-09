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
	altimeterSetup();
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
	pinMode(18, INPUT_PULLUP);
	pinMode(19, INPUT_PULLUP);

	Wire.begin();
	// 1MHz data rate
	Wire.setClock(1000000);

	// start the second I2C bus (BMP390), Wire1, on pins 16 and 17
	pinMode(16, INPUT_PULLUP);
	pinMode(17, INPUT_PULLUP);
	Wire1.begin();
	Wire1.setClock(1000000);
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

//#include <Adafruit_BMP3XX.h>
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

	/* Read temperature and pressure data iteratively based on data ready interrupt */
	if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
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

/*
// override of Adafruit_BMP3XX::performReading() that avoids updating settings
// unnecessarily, and so is faster

class opt_BMP3XX : public Adafruit_BMP3XX {
public:
	bool performReading();
	bool updateSettings();
};
extern Adafruit_I2CDevice *g_i2c_dev; ///< Global I2C interface pointer


bool opt_BMP3XX::performReading(void) {
	g_i2c_dev = i2c_dev;

	uint8_t op_mode;
	int op_mode_get_result = bmp3_get_op_mode(&op_mode, &the_sensor);
	if (op_mode != BMP3_MODE_NORMAL) {
		Serial.print("bad BMP3 mode of ");
		Serial.print(op_mode, HEX);
		Serial.print(". result code: ");
		Serial.println(op_mode_get_result);
		// set the mode???
		the_sensor.settings.op_mode = BMP3_MODE_NORMAL;
		bmp3_set_op_mode(&the_sensor);
		return false;
	}

	// Variable used to store the compensated data
	struct bmp3_data data;

	// Temperature and Pressure data are read and stored in the bmp3_data instance

	uint8_t sensor_comp = BMP3_TEMP | BMP3_PRESS;
	bool rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);
	if (rslt != BMP3_OK)
		return false;

	// Save the temperature and pressure data
	temperature = data.temperature;
	pressure = data.pressure;

	return true;
}
bool opt_BMP3XX::updateSettings() {
	g_i2c_dev = i2c_dev;
	//Adafruit_BMP3XX::performReading();
	//delay(100);

	int8_t rslt;
	// Used to select the settings user needs to change
	uint16_t settings_sel = 0;
	// Variable used to select the sensor component
	uint8_t sensor_comp = 0;

	// Select the pressure and temperature sensor to be enabled
	the_sensor.settings.temp_en = BMP3_ENABLE;
	settings_sel |= BMP3_SEL_TEMP_EN;
	sensor_comp |= BMP3_TEMP;
	if (_tempOSEnabled) {
		settings_sel |= BMP3_SEL_TEMP_OS;
	}

	the_sensor.settings.press_en = BMP3_ENABLE;
	settings_sel |= BMP3_SEL_PRESS_EN;
	sensor_comp |= BMP3_PRESS;
	if (_presOSEnabled) {
		settings_sel |= BMP3_SEL_PRESS_OS;
	}

	if (_filterEnabled) {
		settings_sel |= BMP3_SEL_IIR_FILTER;
	}

	if (_ODREnabled) {
		settings_sel |= BMP3_SEL_ODR;
	}

	// set interrupt to data ready
	// settings_sel |= BMP3_DRDY_EN_SEL | BMP3_LEVEL_SEL | BMP3_LATCH_SEL;

	// Set the desired sensor configuration
#ifdef BMP3XX_DEBUG
	Serial.println("Setting sensor settings");
#endif
	rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);

	if (rslt != BMP3_OK)
		return false;

	// Set the power mode
	the_sensor.settings.op_mode = BMP3_MODE_NORMAL;
#ifdef BMP3XX_DEBUG
	Serial.println(F("Setting power mode"));
#endif
	rslt = bmp3_set_op_mode(&the_sensor);
	if (rslt != BMP3_OK)
		return false;
	return true;
}



#define SEALEVELPRESSURE_HPA (1013.25)

opt_BMP3XX bmp;

void altimeterSetup() {
  Serial.println("Adafruit BMP388 / BMP390 test");

  while (!bmp.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire1)) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    delay(1000);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);

  bmp.updateSettings();
}

void readAltimeter() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  float atmospheric = bmp.pressure / 100.0F;
  float altitude = 44330.0 * (1.0 - pow(atmospheric / SEALEVELPRESSURE_HPA, 0.1903));

  telem_pressureTemp390(bmp.pressure / 100.0, bmp.temperature, altitude);

  //Serial.print("Temp = ");
  //Serial.print(bmp.temperature);
  //Serial.print(" *C. ");

  //Serial.print("Pressure = ");
  //Serial.print(bmp.pressure / 100.0);
  //Serial.println(" hPa. ");

  //Serial.print("Approx. Altitude = ");
  //Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  //Serial.println(" m");
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
	}
}*/

