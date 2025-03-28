#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#define ACCEL_CONFIG 0x1C     // Accelerometer configuration address
#define GYRO_CONFIG  0x1B     // Gyro configuration address

#define PWR_MGMT 0x6B

class TwoWire;

/**
 * @struct XYZ
 * @brief Simple struct for referencing X, Y, Z floating point values.
 */
struct XYZ {
    float x; // The X-axis or X-dimension component.
    float y; // The Y-axis or Y-dimension component.
    float z; // The Z-axis or Z-dimension component.
};

/**
 * @class Mpu6050
 * @brief Generic driver for the MPU-6050 Inertial Measurement Unit
 */
class Mpu6050 {
    private:
        TwoWire& wire;
        unsigned char deviceAddress;
        char accelSensitivity;
        char gyroSensitivity;
        static const unsigned char DEFAULT_ADDRESS = 0x68;
        XYZ accelOffset;
        XYZ gyroOffset;

        void _transmit(unsigned char, unsigned char, bool);
        void _transmit(unsigned char, bool);

    public:
        /**
         * @brief Constructor for MPU-6050 driver instance. Uses default address (0x68).
         * @param wireInstance A pre-defined instance of a Wire class.
         */
        Mpu6050(TwoWire& wireInstance);

        /**
         * @brief Constructor for MPU-6050 driver instance.
         * @param wireInstance A pre-instantiated (but not pre-initialized) instance of a Wire class.
         * @param lsb The least-significant bit of the MPU-6050's I2C device address (0x68 when false, 0x69 when true).
         */
        Mpu6050(TwoWire& wireInstance, bool lsb);

        /**
         * @brief Constructor for MPU-6050 driver instance.
         * @param wireInstance A pre-instantiated (but not pre-initialized) instance of a Wire class.
         * @param address The 8-bit I2C address of the MPU-6050 (normally 0x68 or 0x69).
         */
        Mpu6050(TwoWire& wireInstance, unsigned char address);
        
        /**
         * @brief Initializes handshake with this MPU-6050 device. Uses level 0 for accelerometer and gyroscope sensitivity (2g, 250rad/s).
         * @return true if handshake was successful, otherwise false.
         */
        bool init();

        /**
         * @brief Initializes handshake with this MPU-6050 device.
         * @param accelSensitivity sensitivity level for this device's accelerometer readings (0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g).
         * @param gyroSensitivity sensitivity level for this device's gyroscope readings (0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s).
         * @return true if handshake was successful, otherwise false.
         */
        bool init(char, char);

        // TODO this
        // bool checkConnection();

        /**
         * @brief Runs a calibration procedure for the accelerometer and gyroscope.
         */
        void calibrate();

        /**
         * @brief Requests and returns this device's accelerometer readings in the X, Y, and Z dimensions.
         * @param xyz The address of an instance of an XYZ struct to copy readings into.
         * @return The referenced XYZ instance.
         */
        XYZ requestAccelData(XYZ&);

        /**
         * @brief Requests and returns this device's accelerometer readings in the X, Y, and Z dimensions.
         * @return An XYZ instance containing the accelerometer readings.
         */
        XYZ requestAccelData();

        /**
         * @brief Requests and returns this device's gyroscope readings along the X, Y, and Z axis.
         * @param xyz The address of an instance of an XYZ struct to copy readings into.
         * @return The referenced XYZ instance.
         */
        XYZ requestGyroData(XYZ&);

        /**
         * @brief Requests and returns this device's gyroscope readings along the X, Y, and Z axis.
         * @return An XYZ instance containing the gyroscope readings.
         */
        XYZ requestGyroData();
    };

#endif // MPU6050_DRIVER_H
