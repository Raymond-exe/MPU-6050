#include "mpu6050.h"
#include <Wire.h>  // Include here, but not in the header

Mpu6050::Mpu6050(TwoWire& wireInstance) : wire(wireInstance)  {
    this->wire = wireInstance;
    this->deviceAddress = DEFAULT_ADDRESS;
}

Mpu6050::Mpu6050(TwoWire& wireInstance, bool lsb) : wire(wireInstance)  {
    this->wire = wireInstance;
    this->deviceAddress = (DEFAULT_ADDRESS & (lsb ? 0x01 : 0x00));
}

Mpu6050::Mpu6050(TwoWire& wireInstance, unsigned char address) : wire(wireInstance)  {
    this->wire = wireInstance;
    this->deviceAddress = address;
}

bool Mpu6050::init(void) {
    return this->init(0, 0);
}

bool Mpu6050::init(char accelSensitivity, char gyroSensitivity) {
    this->accelSensitivity = accelSensitivity;
    this->gyroSensitivity = gyroSensitivity;

    this->accelOffset.x = 0;
    this->accelOffset.y = 0;
    this->accelOffset.z = 0;

    this->gyroOffset.x = 0;
    this->gyroOffset.y = 0;
    this->gyroOffset.z = 0;

    this->wire.begin();
    this->_transmit(PWR_MGMT, 0x00, true);
    this->_transmit(ACCEL_CONFIG, (accelSensitivity) << 3, true);
    this->_transmit(GYRO_CONFIG, (gyroSensitivity) << 3, true);
    return true; // TODO verify connection
}

void Mpu6050::calibrate() {
    // Gyroscope calibration
    this->gyroOffset.x = 0;
    this->gyroOffset.y = 0;
    this->gyroOffset.z = 0;
    XYZ gyroSum, gyroRaw;
    gyroSum.x = 0;
    gyroSum.y = 0;
    gyroSum.z = 0;
    // Run 1024 samples and sum the results
    for (int i = 0; i < 1024; i++) {
        this->requestGyroData(gyroRaw);
        gyroSum.x += gyroRaw.x;
        gyroSum.y += gyroRaw.y;
        gyroSum.z += gyroRaw.z;
        delay(3);
    }
    // Average the results and assign as offset
    this->gyroOffset.x = (gyroSum.x / 1024);
    this->gyroOffset.y = (gyroSum.y / 1024);
    this->gyroOffset.z = (gyroSum.z / 1024);

    // Accelerometer calibration
    this->accelOffset.x = 0;
    this->accelOffset.y = 0;
    this->accelOffset.z = 0;
    XYZ accelSum, accelRaw, gravityOffset;
    accelSum.x = 0;
    accelSum.y = 0;
    accelSum.z = 0;
    // gravityOffset is used to gauge which direction gravity is in
    this->requestAccelData(gravityOffset);
    gravityOffset.x = round(gravityOffset.x);
    gravityOffset.y = round(gravityOffset.y);
    gravityOffset.z = round(gravityOffset.z);
    // Run 1024 samples and sum the results
    for (int i = 0; i < 1024; i++) {
        this->requestAccelData(accelRaw);
        accelSum.x += (accelRaw.x - gravityOffset.x);
        accelSum.y += (accelRaw.y - gravityOffset.y);
        accelSum.z += (accelRaw.z - gravityOffset.z);
        delay(3);
    }
    // Average the results and assign as offset
    this->accelOffset.x = (accelSum.x / 1024);
    this->accelOffset.y = (accelSum.y / 1024);
    this->accelOffset.z = (accelSum.z / 1024);
}

XYZ Mpu6050::requestAccelData(XYZ& xyz) {
    this->_transmit(0x3B, false); // TODO make accel reg a #define statement
    this->wire.requestFrom(this->deviceAddress, 6, true); 
    xyz.x = (this->wire.read() << 8 | this->wire.read()) / 16384.0; // TODO get number to divide by for other sensitivity levels
    xyz.y = (this->wire.read() << 8 | this->wire.read()) / 16384.0;
    xyz.z = (this->wire.read() << 8 | this->wire.read()) / 16384.0;

    // Subtract the offsets (assigned upon calibration)
    xyz.x -= this->accelOffset.x;
    xyz.y -= this->accelOffset.y;
    xyz.z -= this->accelOffset.z;

    // Fix negative range
    if (xyz.x > 2) xyz.x -= 4;
    if (xyz.y > 2) xyz.y -= 4;
    if (xyz.z > 2) xyz.z -= 4;
    return xyz;
}

XYZ Mpu6050::requestAccelData() {
    XYZ* xyz = new XYZ;
    return this->requestAccelData(*xyz);
}

XYZ Mpu6050::requestGyroData(XYZ& xyz) {
    this->_transmit(0x43, false); // TODO make gyro reg a #define statement
    this->wire.requestFrom(this->deviceAddress, 6, true);  
    xyz.x = (int16_t)(this->wire.read() << 8 | this->wire.read()); // 0x43, 0x44 (GYRO_XOUT_H, GYRO_XOUT_L)
    xyz.y = (int16_t)(this->wire.read() << 8 | this->wire.read()); // 0x45, 0x46 (GYRO_YOUT_H, GYRO_YOUT_L)
    xyz.z = (int16_t)(this->wire.read() << 8 | this->wire.read()); // 0x47, 0x48 (GYRO_ZOUT_H, GYRO_ZOUT_L)

    // Subtract the offsets (assigned upon calibration)
    xyz.x -= this->gyroOffset.x;
    xyz.y -= this->gyroOffset.y;
    xyz.z -= this->gyroOffset.z;

    xyz.x = (xyz.x / 131); // TODO: why 131?
    xyz.y = (xyz.y / 131);
    xyz.z = (xyz.z / 131);

    return xyz;
}

XYZ Mpu6050::requestGyroData(void) {
    XYZ* xyz = new XYZ;
    return this->requestGyroData(*xyz);
}

void Mpu6050::_transmit(unsigned char addressTo, unsigned char value, bool sendStop) {
    this->wire.beginTransmission(this->deviceAddress);
    this->wire.write(addressTo);
    this->wire.write(value);
    this->wire.endTransmission(sendStop);
}

void Mpu6050::_transmit(unsigned char dataFrom, bool sendStop) {
    this->wire.beginTransmission(this->deviceAddress);
    this->wire.write(dataFrom);
    this->wire.endTransmission(sendStop);
}
