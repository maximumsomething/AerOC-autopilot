#pragma once
void setupAllComms();

void telemSerialSetup();
void usbSerialSetup();
void i2cSetup();
void imuSetup();

void i2cScan();

void printImuData();

struct RawImuData {
  float accelx, accely, accelz;
  float gyrox, gyroy, gyroz;
};

RawImuData readImu();
