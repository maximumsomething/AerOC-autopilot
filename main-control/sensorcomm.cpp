#include "sensorcomm.h"
#include "telemetry.h"
//#include "mpu9250.h"
#include "mpu6500.h"


void setupAllComms() {
  usbSerialSetup();
  telemSerialSetup();
  i2cSetup();
  //i2cScan();
  imuSetup();
}


void usbSerialSetup() {
  Serial.begin(115200);
  while(!Serial) { delay(5); }
}

void telemSerialSetup() {
  Serial1.begin(9600);
  while(!Serial1) { delay(5); }
}

void i2cSetup() {
  //Start the I2C bus
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
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

/*
//Mpu9250 object, SPI bus, CS on pin 10 
bfs::Mpu9250 imu(&SPI, 10);

void imuSetup() {
  // Serial to display data
  Serial.begin(115200);
  while(!Serial) {}
  // Start the SPI bus
  SPI.begin();
  // Initialize and configure IMU
  while (!imu.Begin()) {
    
    Serial.println("Error initializing communication with IMU");
    delay(1000);
  }
  // Set the sample rate divider
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
}*/



// Mpu9250 object
//bfs::Mpu9250 imu;
bfs::Mpu6500 imu;
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
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) { delay(1000); }
  }
}



void printImuData() {
  // Check if data read
  if (imu.Read()) {
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
    /*Serial.print(imu.mag_x_ut());
    Serial.print("\t");
    Serial.print(imu.mag_y_ut());
    Serial.print("\t");
    Serial.print(imu.mag_z_ut());*/
    Serial.print("\t");
    Serial.print(imu.die_temp_c());
    Serial.print("\n");
  }
}
/*
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

void setup() {
  wiresetup();
  unsigned status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
}

void loop() {
  
}
*/
/*
#include <MPU9250_asukiaaa.h>
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
