//==============================================================================
// File: LSM6DSO32.cpp
// Robert Parker
// 8/27/2022
// Version: 1.0
//
// This file along with LSM6DSO32.h defines a class used for interfacing with
// the LSM6DSO32 Accelerometer/Gyro MEMS sensor chip from STMicroelectronis.
// This class wraps the STMicroelectronis provided LSM6DSO32 API functions.
// This implelmenetation supports only a subset of LSM6DSO32 functionality.
// Excluded functionality includes:
//  1. I2C digital interface is not supported. Only SPI is supported.
//  2. FIFO functions are not supported.
//  3. Interrupts are not supported.
//  4. Event detection (e.g. double tap) is not supported
//  5. Sensor hub features are not supported
//==============================================================================

#include "Arduino.h"
#include "LSM6DSO32.h"
#include "lsm6dso32-pid-master/lsm6dso32_reg.c" // LSM6DSO32 API functions

// The chip select pin used for SPI communication with the sensor
// It needs to be a global variable because the static functions used by the
// bmp3 API do not have access to the private class data member _cs
uint8_t chipSelect = 0;

// Hardware interface functions used by the LSM6DSO32 API for SPI digital interface
static int32_t spi_read(void *spiPtr, uint8_t regAddr, uint8_t *regData, uint16_t len);
static int32_t spi_write(void *spiPtr, uint8_t regAddr, const uint8_t *regData, uint16_t len);

//=========================================================================
// Constructor
//=========================================================================
// Currently a do-nothing constructor.
//
// Parameters: None
// Return:  A new LSM6DSO32 instance object
//==========================================================================
LSM6DSO32::LSM6DSO32(void) {}

//=========================================================================
// begin
//=========================================================================
// Initializes the class object and sensor before first use. Calling this
// method will reset the sensor. Following the call to begin() the sensor
// will be configured as follows:
//  Accelerometer full scale range: +/-4 g
//  Accelerometer output data rate: 52 Hz
//  Accelerometer power mode: High performance mode
//  Accelerometer low-pass filter 2 bandwidth set to ODR/2
//  Accelerometer low-pass filter 2 disabled
//  Accelerometer high-pass filter disabled
//  Accelerometer offset correction disabled
//  Gyro full scale range:  +/-125 dps
//  Gyro output data rate:  52 Hz
//  Gyro power mode: High performance mode
//  Gyro low-pass filter bandwidth set to default (ODR dependent)
//  Gyro high-pass filter disabled
//
// Parameters:
//   cs - The processor pin used to select the sensor durint SPI transactions
//   spiPtr - A pointer to the SPI bus controller object.
//   spiFrequency - The frequency in Hz that the SPI bus clock will be set to
// Return:  0
//==========================================================================
bool LSM6DSO32::begin(uint8_t cs, SPIClass *spiPtr, uint32_t spiFrequency)
{
    chipSelect = cs;    // Initialize the global chipSelect variable
    
    // Set the chipSelect pin HIGH
    pinMode(chipSelect, OUTPUT);
    digitalWrite(chipSelect, HIGH);
    
    //Initialize the interface to the LSM6DSO32 API
    sensorAPI_intf.write_reg = &spi_write;
    sensorAPI_intf.read_reg = &spi_read;
    sensorAPI_intf.handle = spiPtr;
    
    return _init();
}

//=========================================================================
// _init (Private method)
//=========================================================================
// This is a helper method for initialization of the sensor
//
// Parameters: None
// Return:  False if an error occurs during initialization. True othewise.
//==========================================================================
bool LSM6DSO32::_init(void) {
    uint8_t chipID;
    lsm6dso32_device_id_get(&sensorAPI_intf, &chipID);  // LSM6DSO32 API call
    
    // make sure we're talking to the right chip
    if (chipID != LSM6DSO32_CHIP_ID) {
#ifdef LSM6DSO32_DEBUG
        Serial.println("Bad chip ID");
        Serial.println(chipID, HEX);
#endif
        return false;
    }
    
    // Perform a software reset of the sensor
    _reset();
    
    // Configure sensor for Block Data Update (BDU)
    lsm6dso32_block_data_update_set(&sensorAPI_intf, 1);    // LSM6DSO32 API call
    
    // Disable I3C interface
    lsm6dso32_i3c_disable_set(&sensorAPI_intf, LSM6DSO32_I3C_DISABLE);  // LSM6DSO32 API call
    
    // Disable the I2C interface
    lsm6dso32_i2c_interface_set(&sensorAPI_intf, LSM6DSO32_I2C_DISABLE);    // LSM6DSO32 API call
    
    // Enable accelerometer with 52 Hz ODR and +/- 4g measurement rande
    setAccelDataRate(LSM6DSO32_ACCEL_52_HZ);
    setAccelRange(LSM6DSO32_RANGE_4G);
    
    // Update the local store
    accelRange = LSM6DSO32_RANGE_4G;
    switch (accelRange) {
        case LSM6DSO32_RANGE_4G:
            accelerationScaleFactor = 0.122;
            break;
        case LSM6DSO32_RANGE_8G:
            accelerationScaleFactor = 0.244;
            break;
        case LSM6DSO32_RANGE_16G:
            accelerationScaleFactor = 0.488;
            break;
        case LSM6DSO32_RANGE_32G:
            accelerationScaleFactor = 0.976;
            break;
    }

    // Enable gyro with 52 ODR and +/-125 dps measurement range
    setGyroDataRate(LSM6DSO32_GYRO_52_HZ);
    setGyroRange(LSM6DSO32_RANGE_125DPS);
    
    // Update the local store
    gyroRange = LSM6DSO32_RANGE_125DPS;
    switch (gyroRange) {
        case LSM6DSO32_RANGE_125DPS:
            rateScaleFactor = 4.375;
            break;
        case LSM6DSO32_RANGE_250DPS:
            rateScaleFactor = 8.75;
            break;
        case LSM6DSO32_RANGE_500DPS:
            rateScaleFactor = 17.5;
            break;
        case LSM6DSO32_RANGE_1000DPS:
            rateScaleFactor = 35.0;
            break;
        case LSM6DSO32_RANGE_2000DPS:
            rateScaleFactor = 70.0;
            break;
    }
    
    return true;
}

//=========================================================================
// _reset (Private method)
//=========================================================================
// This is a helper method for initialization of the sensor. Performs a
// software reset of the sensor.
//
// Parameters: None
// Return:  None
//==========================================================================
void LSM6DSO32::_reset(void) {
    // Set the SW_RESET bit to 1 to start software reset of the sensor
    lsm6dso32_reset_set(&sensorAPI_intf, 1);    // LSM6DSO32 API call
    
    // Wait until the reset operation is complete
    uint8_t resetBitVal = 1;
    while(resetBitVal == 1) {
        delay(5);
        lsm6dso32_reset_get(&sensorAPI_intf, &resetBitVal); // LSM6DSO32 API call
    }
}


//=========================================================================
// readSensorData
//=========================================================================
// Reads the current data values for temperature, acceleration, and rate
// from the sensor. Normalizes the values and stores them in the local
// variables. Data are stored in degrees C, G's, and degrees per second.
// Parameters: None
// Return:  None
//==========================================================================
void LSM6DSO32::readSensorData(void)
{
    uint8_t rawSensorRegisterData[13];
    lsm6dso32_read_reg(&sensorAPI_intf, LSM6DSO32_OUT_TEMP_H, rawSensorRegisterData, 13);
    
    // Temperature
    temperature = (int8_t)rawSensorRegisterData[0] + 25.0;
    
    // Angular Rate
    int16_t rawData = 0;
    rawData = rawSensorRegisterData[2] << 8 | rawSensorRegisterData[1];
    xAxisRate = rawData * rateScaleFactor / 1000.0;
    rawData = rawSensorRegisterData[4] << 8 | rawSensorRegisterData[3];
    yAxisRate = rawData * rateScaleFactor / 1000.0;
    rawData = rawSensorRegisterData[6] << 8 | rawSensorRegisterData[5];
    zAxisRate = rawData * rateScaleFactor / 1000.0;
    
    // Acceleration
    rawData = rawSensorRegisterData[8] << 8 | rawSensorRegisterData[7];
    xAxisAccel = rawData * accelerationScaleFactor / 1000.0;
    rawData = rawSensorRegisterData[10] << 8 | rawSensorRegisterData[9];
    yAxisAccel = rawData * accelerationScaleFactor / 1000.0;
    rawData = rawSensorRegisterData[12] << 8 | rawSensorRegisterData[11];
    zAxisAccel = rawData * accelerationScaleFactor / 1000.0;
    
}

//=========================================================================
// readAccelData
//=========================================================================
// Reads the current data values for acceleration from the sensor. Normalizes
// the values and stores them in the local variables. Data are stored in G's.
// Parameters: None
// Return:  None
//==========================================================================
void LSM6DSO32::readAccelData(void)
{
    int16_t rawAccelData[3];
    lsm6dso32_acceleration_raw_get(&sensorAPI_intf, rawAccelData);
    
    xAxisAccel = rawAccelData[0] * accelerationScaleFactor / 1000.0;
    yAxisAccel = rawAccelData[1] * accelerationScaleFactor / 1000.0;
    zAxisAccel = rawAccelData[2] * accelerationScaleFactor / 1000.0;
}

//=========================================================================
// readGyroData
//=========================================================================
// Reads the current data values for rate from the sensor. Normalizes
// the values and stores them in the local variables. Data are stored in
// degrees per second.
// Parameters: None
// Return:  None
//==========================================================================
void LSM6DSO32::readGyroData(void)
{
    int16_t rawGyroData[3];
    lsm6dso32_angular_rate_raw_get(&sensorAPI_intf, rawGyroData);
    
    xAxisRate = rawGyroData[0] * rateScaleFactor / 1000.0;
    yAxisRate = rawGyroData[1] * rateScaleFactor / 1000.0;
    zAxisRate = rawGyroData[2] * rateScaleFactor / 1000.0;
}

//=========================================================================
// readTempData
//=========================================================================
// Reads the current data value for temperature from the sensor. Normalizes
// the value and stores it in the local variable (temperature). Data are
// stored in degrees C.
// Parameters: None
// Return:  None
//==========================================================================
void LSM6DSO32::readTempData(void)
{
    uint8_t rawTempData;
    lsm6dso32_read_reg(&sensorAPI_intf, LSM6DSO32_OUT_TEMP_H, &rawTempData, 1);
    temperature = (int8_t)rawTempData + 25.0;
}

//=========================================================================
// getTemperature
//=========================================================================
// Returns the most recently read sensor temperature value in degrees C.
// This value is stored locally. It is not read from the sensor.
// Parameters: None
// Return:  The last read temperature value in degrees C
//==========================================================================
float LSM6DSO32::getTemperature(void)
{
    return temperature;
}

//=========================================================================
// getXAxisAccel
//=========================================================================
// Returns the most recently read sensor X axis acceleration value in Gs.
// This value is stored locally. It is not read from the sensor.
// Parameters: None
// Return:  The last read X axis acceleration value in Gs
//==========================================================================
float LSM6DSO32::getXAxisAccel(void)
{
    return xAxisAccel;
}

//=========================================================================
// getYAxisAccel
//=========================================================================
// Returns the most recently read sensor Y axis acceleration value in Gs.
// This value is stored locally. It is not read from the sensor.
// Parameters: None
// Return:  The last read Y axis acceleration value in Gs
//==========================================================================
float LSM6DSO32::getYAxisAccel(void)
{
    return yAxisAccel;
}

//=========================================================================
// getZAxisAccel
//=========================================================================
// Returns the most recently read sensor Z axis acceleration value in Gs.
// This value is stored locally. It is not read from the sensor.
// Parameters: None
// Return:  The last read Z axis acceleration value in Gs
//==========================================================================
float LSM6DSO32::getZAxisAccel(void)
{
    return zAxisAccel;
}

//=========================================================================
// getXAxisRate
//=========================================================================
// Returns the most recently read sensor X axis rate value in degrees per
// second. This value is stored locally. It is not read from the sensor.
// Parameters: None
// Return:  The last read X axis rate value in degrees per second
//==========================================================================
float LSM6DSO32::getXAxisRate(void)
{
    return xAxisRate;
}

//=========================================================================
// getYAxisRate
//=========================================================================
// Returns the most recently read sensor Y axis rate value in degrees per
// second. This value is stored locally. It is not read from the sensor.
// Parameters: None
// Return:  The last read Y axis rate value in degrees per second
//==========================================================================
float LSM6DSO32::getYAxisRate(void)
{
    return yAxisRate;
}

//=========================================================================
// getZAxisRate
//=========================================================================
// Returns the most recently read sensor Z axis rate value in degrees per
// second. This value is stored locally. It is not read from the sensor.
// Parameters: None
// Return:  The last read Z axis rate value in degrees per second
//==========================================================================
float LSM6DSO32::getZAxisRate(void)
{
    return zAxisRate;
}

//=========================================================================
// getAccelRange
//=========================================================================
// Returns the current accelerometer sensor full scale range setting.
// Parameters: None
// Return:  Current full scale range setting
//==========================================================================
lsm6dso32_accel_range_t LSM6DSO32::getAccelRange(void) {
    
    lsm6dso32_fs_xl_t range;
    lsm6dso32_xl_full_scale_get(&sensorAPI_intf, &range);
    
    // Update the local store
    accelRange = (lsm6dso32_accel_range_t)range;
    switch (accelRange) {
        case LSM6DSO32_RANGE_4G:
            accelerationScaleFactor = 0.122;
            break;
        case LSM6DSO32_RANGE_8G:
            accelerationScaleFactor = 0.244;
            break;
        case LSM6DSO32_RANGE_16G:
            accelerationScaleFactor = 0.488;
            break;
        case LSM6DSO32_RANGE_32G:
            accelerationScaleFactor = 0.976;
            break;
    }
    
    return (lsm6dso32_accel_range_t)range;
}

//=========================================================================
// setAccelRange
//=========================================================================
// Sets the sensor accelerometer full-scale measurement range to the
// specified value.
// Parameters:
//   range - The full-scaler accelerometer measurement range to be set.
//           Valid values are LSM6DSO32_4g, LSM6DSO32_8g, LSM6DSO32_16g,
//           and LSM6DSO32_32g.
// Return:  0
//==========================================================================
void LSM6DSO32::setAccelRange(lsm6dso32_accel_range_t range)
{
    lsm6dso32_xl_full_scale_set(&sensorAPI_intf, (lsm6dso32_fs_xl_t)range);    // LSM6DSO32 API call
    
    // Update the local store
    accelRange = (lsm6dso32_accel_range_t)range;
    switch (accelRange) {
        case LSM6DSO32_RANGE_4G:
            accelerationScaleFactor = 0.122;
            break;
        case LSM6DSO32_RANGE_8G:
            accelerationScaleFactor = 0.244;
            break;
        case LSM6DSO32_RANGE_16G:
            accelerationScaleFactor = 0.488;
            break;
        case LSM6DSO32_RANGE_32G:
            accelerationScaleFactor = 0.976;
            break;
    }
    
    delay(20);
}

//=========================================================================
// getAccelDataRate
//=========================================================================
// Returns the current accelerometer sensor output data rate (ODR) setting.
// Parameters: None
// Return:  Current accelerometer ODR setting
//==========================================================================
lsm6dso32_accel_data_rate_t LSM6DSO32::getAccelDataRate(void)
{
    lsm6dso32_odr_xl_t dataRate;
    lsm6dso32_xl_data_rate_get(&sensorAPI_intf, &dataRate);
    lsm6dso32_accel_data_rate_t rate;
    
    switch (dataRate) {
        case LSM6DSO32_XL_ODR_OFF:
            rate = LSM6DSO32_ACCEL_SHUTDOWN;
            break;
        case LSM6DSO32_XL_ODR_6Hz5_LOW_PW:
        case LSM6DSO32_XL_ODR_6Hz5_ULTRA_LOW_PW:
            rate = LSM6DSO32_ACCEL_6_5_HZ;
            break;
        case LSM6DSO32_XL_ODR_12Hz5_LOW_PW:
        case LSM6DSO32_XL_ODR_12Hz5_HIGH_PERF:
        case LSM6DSO32_XL_ODR_12Hz5_ULTRA_LOW_PW:
            rate = LSM6DSO32_ACCEL_12_5_HZ;
            break;
        case LSM6DSO32_XL_ODR_26Hz_LOW_PW:
        case LSM6DSO32_XL_ODR_26Hz_HIGH_PERF:
        case LSM6DSO32_XL_ODR_26Hz_ULTRA_LOW_PW:
            rate = LSM6DSO32_ACCEL_26_HZ;
            break;
        case LSM6DSO32_XL_ODR_52Hz_LOW_PW:
        case LSM6DSO32_XL_ODR_52Hz_HIGH_PERF:
        case LSM6DSO32_XL_ODR_52Hz_ULTRA_LOW_PW:
            rate = LSM6DSO32_ACCEL_52_HZ;
            break;
        case LSM6DSO32_XL_ODR_104Hz_NORMAL_MD:
        case LSM6DSO32_XL_ODR_104Hz_HIGH_PERF:
        case LSM6DSO32_XL_ODR_104Hz_ULTRA_LOW_PW:
            rate = LSM6DSO32_ACCEL_104_HZ;
            break;
        case LSM6DSO32_XL_ODR_208Hz_NORMAL_MD:
        case LSM6DSO32_XL_ODR_208Hz_HIGH_PERF:
        case LSM6DSO32_XL_ODR_208Hz_ULTRA_LOW_PW:
            rate = LSM6DSO32_ACCEL_208_HZ;
            break;
        case LSM6DSO32_XL_ODR_417Hz_HIGH_PERF:
            rate = LSM6DSO32_ACCEL_417_HZ;
            break;
        case LSM6DSO32_XL_ODR_833Hz_HIGH_PERF:
            rate = LSM6DSO32_ACCEL_833_HZ;
            break;
        case LSM6DSO32_XL_ODR_1667Hz_HIGH_PERF:
            rate = LSM6DSO32_ACCEL_1667_HZ;
            break;
        case LSM6DSO32_XL_ODR_3333Hz_HIGH_PERF:
            rate = LSM6DSO32_ACCEL_3333_HZ;
            break;
        case LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF:
            rate = LSM6DSO32_ACCEL_6667_HZ;
            break;
    }
    return rate;
}

//=========================================================================
// setAccelDataRate
//=========================================================================
// Sets the sensor accelerometer data rate and power mode to the specified
// values. Note that not all combinations of odr and powerMode are valid;
// refer to the data sheet for valid combinations. This method will return
// an error code if an invalid combination is requested.
// Parameters:
//   odr - The output data rate of the accelerometer sensor. Valid values are
//         LSM6DSO32_ACCEL_SHUTDOWN, LSM6DSO32_ACCEL_6_5_HZ, LSM6DSO32_ACCEL_12_5_HZ,
//         LSM6DSO32_ACCEL_26_HZ, LSM6DSO32_ACCEL_52_HZ, LSM6DSO32_ACCEL_104_HZ,
//         LSM6DSO32_ACCEL_208_HZ, LSM6DSO32_ACCEL_417_HZ, LSM6DSO32_ACCEL_833_HZ,
//         LSM6DSO32_ACCEL_1667_HZ, LSM6DSO32_ACCEL_3333_HZ, and
//         LSM6DSO32_ACCEL_6667_HZ.
//  powerMode - The power mode setting for the accelerometer sensor. Valid
//              values are LSM6DSO32_ACCEL_POWER_DOWN, LSM6DSO32_ACCEL_ULTRA_LOW_POWER,
//              LSM6DSO32_ACCEL_LOW_POWER, LSM6DSO32_ACCEL_NORMAL, and
//              LSM6DSO32_ACCEL_HIGH_PERFORMANCE (default value).
// Return:  0 if successfull. An error code otherwise
//==========================================================================
int8_t LSM6DSO32::setAccelDataRate(lsm6dso32_accel_data_rate_t odr, lsm6dso32_accel_power_mode_t powerMode  )
{
    switch (powerMode) {
        case LSM6DSO32_ACCEL_HIGH_PERFORMANCE:
            //This is the most likely case so do it first
            switch (odr) {
                case LSM6DSO32_ACCEL_12_5_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_12Hz5_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_26_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_26Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_52_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_52Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_104_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_104Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_208_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_208Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_417_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_417Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_833_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_833Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_1667_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_1667Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_3333_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_3333Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_ACCEL_6667_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_6667Hz_HIGH_PERF);
                    break;
                default:
                    // Invalid ODR for High Performance Mode
                    return LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS;
            }
        case LSM6DSO32_ACCEL_LOW_POWER:
            switch (odr) {
                case LSM6DSO32_ACCEL_6_5_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_6Hz5_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_12_5_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_12Hz5_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_26_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_26Hz_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_52_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_52Hz_LOW_PW);
                    break;
                default:
                    return LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS;
            }
        case LSM6DSO32_ACCEL_NORMAL:
            switch (odr) {
                case LSM6DSO32_ACCEL_104_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_104Hz_NORMAL_MD);
                    break;
                case LSM6DSO32_ACCEL_208_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_208Hz_NORMAL_MD);
                    break;
                default:
                    return LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS;
            }
        case LSM6DSO32_ACCEL_ULTRA_LOW_POWER:
            // First we need to turn off the gyro. Ultra Low Power mode
            // only works with the gyro turn off
            lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_OFF);
            switch (odr) {
                case LSM6DSO32_ACCEL_6_5_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_6Hz5_ULTRA_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_12_5_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_12Hz5_ULTRA_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_26_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_26Hz_ULTRA_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_52_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_52Hz_ULTRA_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_104_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_104Hz_ULTRA_LOW_PW);
                    break;
                case LSM6DSO32_ACCEL_208_HZ:
                    lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_208Hz_ULTRA_LOW_PW);
                    break;
                default:
                    return LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS;
            }
        case LSM6DSO32_ACCEL_POWER_DOWN:
            lsm6dso32_xl_data_rate_set(&sensorAPI_intf, LSM6DSO32_XL_ODR_OFF);
            break;
    }
    return 0;
}

//=========================================================================
// getGyroRange
//=========================================================================
// Returns the current gyro sensor full scale range setting.
// Parameters: None
// Return:  Current full scale range setting
//==========================================================================
lsm6dso32_gyro_range_t LSM6DSO32::getGyroRange(void) {
    
    lsm6dso32_fs_g_t range;
    lsm6dso32_gy_full_scale_get(&sensorAPI_intf, &range);
    // Update local store
    gyroRange = (lsm6dso32_gyro_range_t)range;
    switch (gyroRange) {
        case LSM6DSO32_RANGE_125DPS:
            rateScaleFactor = 4.375;
            break;
        case LSM6DSO32_RANGE_250DPS:
            rateScaleFactor = 8.75;
            break;
        case LSM6DSO32_RANGE_500DPS:
            rateScaleFactor = 17.5;
            break;
        case LSM6DSO32_RANGE_1000DPS:
            rateScaleFactor = 35.0;
            break;
        case LSM6DSO32_RANGE_2000DPS:
            rateScaleFactor = 70.0;
            break;
    }
    return (lsm6dso32_gyro_range_t)range;
}

//=========================================================================
// setGyroRange
//=========================================================================
// Sets the sensor gyro full-scale measurement range to the specified value.
// Parameters:
//   range - The full-scaler gyro measurement range to be set.
//           Valid values are LSM6DSO32_125dps, LSM6DSO32_250dps
//           LSM6DSO32_500dps, LSM6DSO32_1000dps, and LSM6DSO32_2000dps.
// Return:  0
//==========================================================================
void LSM6DSO32::setGyroRange(lsm6dso32_gyro_range_t range)
{
    lsm6dso32_gy_full_scale_set(&sensorAPI_intf, (lsm6dso32_fs_g_t)range);    //LSM6DSO32 API call
    
    // Update the local store
    gyroRange = range;
    switch (gyroRange) {
        case LSM6DSO32_RANGE_125DPS:
            rateScaleFactor = 4.375;
            break;
        case LSM6DSO32_RANGE_250DPS:
            rateScaleFactor = 8.75;
            break;
        case LSM6DSO32_RANGE_500DPS:
            rateScaleFactor = 17.5;
            break;
        case LSM6DSO32_RANGE_1000DPS:
            rateScaleFactor = 35.0;
            break;
        case LSM6DSO32_RANGE_2000DPS:
            rateScaleFactor = 70.0;
            break;
    }
    
    delay(20);
}

//=========================================================================
// getGyroDataRate
//=========================================================================
// Returns the current gyro sensor output data rate (ODR) setting.
// Parameters: None
// Return:  Current gyro ODR setting
//==========================================================================
lsm6dso32_gyro_data_rate_t LSM6DSO32::getGyroDataRate(void)
{
    lsm6dso32_odr_g_t dataRate;
    lsm6dso32_gy_data_rate_get(&sensorAPI_intf, &dataRate);
    lsm6dso32_gyro_data_rate_t rate;
    
    switch (dataRate) {
        case LSM6DSO32_GY_ODR_OFF:
            rate = LSM6DSO32_GYRO_SHUTDOWN;
            break;
        case LSM6DSO32_GY_ODR_12Hz5_HIGH_PERF:
        case LSM6DSO32_GY_ODR_12Hz5_LOW_PW:
            rate = LSM6DSO32_GYRO_12_5_HZ;
            break;
        case LSM6DSO32_GY_ODR_26Hz_HIGH_PERF:
        case LSM6DSO32_GY_ODR_26Hz_LOW_PW:
            rate = LSM6DSO32_GYRO_26_HZ;
            break;
        case LSM6DSO32_GY_ODR_52Hz_HIGH_PERF:
        case LSM6DSO32_GY_ODR_52Hz_LOW_PW:
            rate = LSM6DSO32_GYRO_52_HZ;
            break;
        case LSM6DSO32_GY_ODR_104Hz_HIGH_PERF:
        case LSM6DSO32_GY_ODR_104Hz_NORMAL_MD:
            rate = LSM6DSO32_GYRO_104_HZ;
            break;
        case LSM6DSO32_GY_ODR_208Hz_NORMAL_MD:
        case LSM6DSO32_GY_ODR_208Hz_HIGH_PERF:
            rate = LSM6DSO32_GYRO_208_HZ;
            break;
        case LSM6DSO32_GY_ODR_417Hz_HIGH_PERF:
            rate = LSM6DSO32_GYRO_417_HZ;
            break;
        case LSM6DSO32_GY_ODR_833Hz_HIGH_PERF:
            rate = LSM6DSO32_GYRO_833_HZ;
            break;
        case LSM6DSO32_GY_ODR_1667Hz_HIGH_PERF:
            rate = LSM6DSO32_GYRO_1667_HZ;
            break;
        case LSM6DSO32_GY_ODR_3333Hz_HIGH_PERF:
            rate = LSM6DSO32_GYRO_3333_HZ;
            break;
        case LSM6DSO32_GY_ODR_6667Hz_HIGH_PERF:
            rate = LSM6DSO32_GYRO_6667_HZ;
            break;
    }
    return rate;
}

//=========================================================================
// setGyroDataRate
//=========================================================================
// Sets the sensor gyro data rate and power mode to the specified
// values. Note that not all combinations of odr and powerMode are valid;
// refer to the data sheet for valid combinations. This method will return
// an error code if an invalid combination is requested.
// Parameters:
//   odr - The output data rate of the accelerometer sensor. Valid values are
//         LSM6DSO32_GYRO_SHUTDOWN, LSM6DSO32_GYRO_12_5_HZ, LSM6DSO32_GYRO_26_HZ,
//         LSM6DSO32_GYRO_52_HZ, LSM6DSO32_GYRO_104_HZ, LSM6DSO32_GYRO_208_HZ,
//         LSM6DSO32_GYRO_417_HZ, LSM6DSO32_GYRO_833_HZ, LSM6DSO32_GYRO_1667_HZ,
//         LSM6DSO32_GYRO_3333_HZ, and LSM6DSO32_GYRO_6667_HZ.
//  powerMode - The power mode setting for the accelerometer sensor. Valid
//              values are LSM6DSO32_GYRO_POWER_DOWN, LSM6DSO32_GYRO_LOW_POWER,
//              LSM6DSO32_GYRO_NORMAL, and LSM6DSO32_GYRO_HIGH_PERFORMANCE
//              (default value0.
// Return:  0 if successfull. An error code otherwise
//==========================================================================
int8_t LSM6DSO32::setGyroDataRate(lsm6dso32_gyro_data_rate_t odr, lsm6dso32_gyro_power_mode_t powerMode )
{
    switch (powerMode) {
        case LSM6DSO32_GYRO_HIGH_PERFORMANCE:
            switch (odr) {
                case LSM6DSO32_GYRO_12_5_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_12Hz5_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_26_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_26Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_52_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_52Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_104_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_104Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_208_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_208Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_417_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_417Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_833_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_833Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_1667_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_1667Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_3333_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_3333Hz_HIGH_PERF);
                    break;
                case LSM6DSO32_GYRO_6667_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_6667Hz_HIGH_PERF);
                    break;
                default:
                    return LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS;
            }
        case LSM6DSO32_GYRO_LOW_POWER:
            switch (odr) {
                case LSM6DSO32_GYRO_12_5_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_12Hz5_LOW_PW);
                    break;
                case LSM6DSO32_GYRO_26_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_26Hz_LOW_PW);
                    break;
                case LSM6DSO32_GYRO_52_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_52Hz_LOW_PW);
                    break;
                default:
                    return LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS;
            }
        case LSM6DSO32_GYRO_NORMAL:
            switch (odr) {
                case LSM6DSO32_GYRO_104_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_104Hz_NORMAL_MD);
                    break;
                case LSM6DSO32_GYRO_208_HZ:
                    lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_208Hz_NORMAL_MD);
                    break;
                default:
                    return LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS;
            }
        case LSM6DSO32_GYRO_POWER_DOWN:
            lsm6dso32_gy_data_rate_set(&sensorAPI_intf, LSM6DSO32_GY_ODR_OFF);
            break;
    }
    return 0;
}

//=========================================================================
// enableAccelLowPassFilter2
//=========================================================================
// Enables (or disables) low pass filter 2 in the accelerometer output data
// path.
// Parameters:
//  enable - If true the low pass filter is enabled. It is disabled othewise.
//  bandwidth - The desired bandwidth setting for low pass filter 2. Valid
//              values are LSM6DSO32_ACCEL_LPF2_DIV4, LSM6DSO32_ACCEL_LPF2_DIV10,
//              LSM6DSO32_ACCEL_LPF2_DIV20, LSM6DSO32_ACCEL_LPF2_DIV45,
//              LSM6DSO32_ACCEL_LPF2_DIV100, LSM6DSO32_ACCEL_LPF2_DIV200,
//              LSM6DSO32_ACCEL_LPF2_DIV400, and LSM6DSO32_ACCEL_LPF2_DIV800.
// Return:  None
//==========================================================================
void LSM6DSO32::enableAccelLowPassFilter2(bool enable, lsm6dso32_accel_lpf2_bw_t bandwidth)
{
    if (enable) {
        switch (bandwidth) {
            case LSM6DSO32_ACCEL_LPF2_DIV4:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_PATH_DISABLE_ON_OUT);
                break;
            case LSM6DSO32_ACCEL_LPF2_DIV10:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_LP_ODR_DIV_10);
                break;
            case LSM6DSO32_ACCEL_LPF2_DIV20:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_LP_ODR_DIV_20);
                break;
            case LSM6DSO32_ACCEL_LPF2_DIV45:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_LP_ODR_DIV_45);
                break;
            case LSM6DSO32_ACCEL_LPF2_DIV100:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_LP_ODR_DIV_100);
                break;
            case LSM6DSO32_ACCEL_LPF2_DIV200:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_LP_ODR_DIV_200);
                break;
            case LSM6DSO32_ACCEL_LPF2_DIV400:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_LP_ODR_DIV_400);
                break;
            case LSM6DSO32_ACCEL_LPF2_DIV800:
                lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_LP_ODR_DIV_800);
                break;
        }
        // Enable Low Pass Filter 2
        lsm6dso32_xl_filter_lp2_set(&sensorAPI_intf, 1);
    } else {
        // Disable Low Pass Filter 2
        lsm6dso32_xl_filter_lp2_set(&sensorAPI_intf, 0);
    }
}

//=========================================================================
// enableAccelHighPassFilter
//=========================================================================
// Enables (or disables) the high pass filter in the accelerometer output data
// path. Note: Enabling the high pass filter will bypass low pass filter 2.
// Parameters:
//  enable - If true the low pass filter is enabled. It is disabled othewise.
//  bandwidth - The desired bandwidth setting for the high pass filter. Valid
//              values are LSM6DSO32_ACCEL_HPF_DIV4 (only when refModeEnable = false,
//              LSM6DSO32_ACCEL_HPF_DIV10, LSM6DSO32_ACCEL_HPF_DIV20,
//              LSM6DSO32_ACCEL_HPF_DIV45, LSM6DSO32_ACCEL_HPF_DIV100,
//              LSM6DSO32_ACCEL_HPF_DIV200, LSM6DSO32_ACCEL_HPF_DIV400, and
//              LSM6DSO32_ACCEL_HPF_DIV800.
//  refModeEnable - If true then the high pass filter reference mode is enabled.
//                  If false (default) it is disabled.
// Return:  None
//==========================================================================
void LSM6DSO32::enableAccelHighPassFilter(bool enable,
                                                   lsm6dso32_accel_hpf_bw_t bandwidth,
                                                   bool refModeEnable)
{
    if (enable) {
        if (!refModeEnable) {
            switch (bandwidth) {
                case LSM6DSO32_ACCEL_HPF_DIV4:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_SLOPE_ODR_DIV_4);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV10:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_ODR_DIV_10);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV20:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_ODR_DIV_20);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV45:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_ODR_DIV_45);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV100:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_ODR_DIV_100);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV200:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_ODR_DIV_200);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV400:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_ODR_DIV_400);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV800:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_ODR_DIV_800);
                    break;
            }
        } else {
            switch (bandwidth) {
                case LSM6DSO32_ACCEL_HPF_DIV10:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_REF_MD_ODR_DIV_10);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV20:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_REF_MD_ODR_DIV_20);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV45:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_REF_MD_ODR_DIV_45);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV100:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_REF_MD_ODR_DIV_100);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV200:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_REF_MD_ODR_DIV_200);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV400:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_REF_MD_ODR_DIV_400);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV800:
                    lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_REF_MD_ODR_DIV_800);
                    break;
                case LSM6DSO32_ACCEL_HPF_DIV4:
                    break;
            }
        }
    } else {
        lsm6dso32_xl_hp_path_on_out_set(&sensorAPI_intf, LSM6DSO32_HP_PATH_DISABLE_ON_OUT);
    }
}

//=========================================================================
// enableAccelOffsetCorrection
//=========================================================================
// Enables (or disables) accelerometer sensor offset correction.
// Parameters:
//  enable - If true offset correction is enabled. It is disabled othewise.
//  weight - The weight of each bit in the offset correction value. Valid values
//           are LSM6DSO32_ACCEL_OFFSET_WEIGHT_976MICRO_G, and
//           LSM6DSO32_ACCEL_OFFSET_WEIGHT_15MILLI_G,
//  xAxisCorrection - The X axis correction value between -127 and +127.
//  yAxisCorrection - The Y axis correction value between -127 and +127.
//  zAxisCorrection - The Z axis correction value between -127 and +127.
// Return:  None
//==========================================================================
void LSM6DSO32::enableAccelOffsetCorrection(bool enable,
                                                     lsm6dso32_accel_offset_corr_t weight,
                                                     int8_t xAxisCorrection,
                                                     int8_t yAxisCorrection,
                                                     int8_t zAxisCorrection)
{
    if (enable) {
        // Set the weight of correction bits
        if (weight == LSM6DSO32_ACCEL_OFFSET_WEIGHT_1MG) {
            lsm6dso32_xl_offset_weight_set(&sensorAPI_intf, LSM6DSO32_LSb_1mg);
        } else {
            lsm6dso32_xl_offset_weight_set(&sensorAPI_intf, LSM6DSO32_LSb_16mg);
        }
        
        // Write the offset correction values
        lsm6dso32_xl_usr_offset_x_set(&sensorAPI_intf, (uint8_t *)&xAxisCorrection);
        lsm6dso32_xl_usr_offset_y_set(&sensorAPI_intf, (uint8_t *)&yAxisCorrection);
        lsm6dso32_xl_usr_offset_z_set(&sensorAPI_intf, (uint8_t *)&zAxisCorrection);
        // Enable offset correction
        lsm6dso32_xl_usr_offset_set(&sensorAPI_intf, 1);
    } else {
        // Disbale offset correction
        lsm6dso32_xl_usr_offset_set(&sensorAPI_intf, 0);
    }
}

//=========================================================================
// enableGyroLowPassFilter
//=========================================================================
// Enables (or disables) the gyro sensor low pass filter.
// Parameters:
//  enable - If true the low pass filter is enabled. It is disabled othewise.
//  bandwidth - The bandwidth setting for the low pass filter. Valid values
//              are LSM6DSO32_GYRO_LPF_LVL0, LSM6DSO32_GYRO_LPF_LVL1,
//              LSM6DSO32_GYRO_LPF_LVL2, LSM6DSO32_GYRO_LPF_LVL3,
//              LSM6DSO32_GYRO_LPF_LVL4, LSM6DSO32_GYRO_LPF_LVL5,
//              LSM6DSO32_GYRO_LPF_LVL6, and LSM6DSO32_GYRO_LPF_LVL7. Actual
//              low pass filter bandwidth is ODR dependent. See Table 60
//              in data sheet.
// Return:  None
//==========================================================================
void LSM6DSO32::enableGyroLowPassFilter(bool enable, lsm6dso32_gyro_lpf_bw_t bandwidth)
{
    if (enable) {
        // Change the bandwidth setting
        lsm6dso32_gy_lp1_bandwidth_set(&sensorAPI_intf, (lsm6dso32_ftype_t)bandwidth);
        // Enable the gyro low pass filter
        lsm6dso32_gy_filter_lp1_set(&sensorAPI_intf, 1);
    } else {
        lsm6dso32_gy_filter_lp1_set(&sensorAPI_intf, 0);
    }
}

//=========================================================================
// enableGyroHighPassFilter
//=========================================================================
// Enables (or disables) the gyro sensor high pass filter.
// Parameters:
//  enable - If true the high pass filter is enabled. It is disabled othewise.
//  bandwidth - The bandwidth setting for the high pass filter. Valid values
//              are LSM6DSO32_GYRO_HPF_0_016HZ, LSM6DSO32_GYRO_HPF_0_065HZ,
//              LSM6DSO32_GYRO_HPF_0_260HZ, and LSM6DSO32_GYRO_HPF_1_04HZ.
// Return:  None
//==========================================================================
void LSM6DSO32::enableGyroHighPassFilter(bool enable, lsm6dso32_gyro_hpf_bw_t bandwidth)
{
    if (enable) {
        lsm6dso32_gy_hp_path_internal_set(&sensorAPI_intf, (lsm6dso32_hpm_g_t)bandwidth);
    } else {
        lsm6dso32_gy_hp_path_internal_set(&sensorAPI_intf, LSM6DSO32_HP_FILTER_NONE);
    }
}


//=========================================================================
// spi_read
//=========================================================================
// This function is called by the LSM6DSO32 API to read data from the sensor.
// First a starting register address is written to the sensor and then the
// data from one or more sensor registers is read back.
// Parameters:
//   spiPtr - A pointer to the SPI bus controller object
//   regAddr - The address of the sensor register from which reading will
//             begin.
//   regData - A pointer to a data buffer where the register data read from
//             the sensor will be stored.
//   len - The number of registers that will be read during the transaction
// Return:  0
//==========================================================================
static int32_t spi_read(void *spiPtr, uint8_t regAddr, uint8_t *regData, uint16_t len)
{
#ifdef LSM6DSO32_DEBUG
    Serial.println("spi_read()");
#endif
    SPIClass *spi = (SPIClass *)spiPtr;
    
    // Begin transaction
    spi->beginTransaction(SPISettings(DEFAULT_SPI_FREQ, MSBFIRST, SPI_MODE0));
    
    // Assert chip select
    digitalWrite(chipSelect, LOW);
    
    // Write register address
    spi->transfer(regAddr | 0x80);
    
    // Read register data
    for (size_t index = 0; index < len; index++) {
        regData[index] = spi->transfer(0xFF);
    }
    
    // Deassert chip select
    digitalWrite(chipSelect, HIGH);
    
    // End transaction
    spi->endTransaction();
    
    return 0;
}

//=========================================================================
// spi_write
//=========================================================================
// This function is called by the LSM3DSO32 API to write data to the sensor.
// First a starting register address is written to the sensor and then one
// or more data bytes are written to consecutive registers in the sensor.
// Parameters:
//   spiPtr - A pointer to the SPI bus controller object
//   regAddr - The address of the sensor register that will receive the
//             first byte of data.
//   regData - A pointer to a data buffer where the register data to be
//              written is stored.
//   len - The number of bytes that will be written
// Return:  0
//==========================================================================
static int32_t spi_write(void *spiPtr, uint8_t regAddr, const uint8_t *regData, uint16_t len)
{
#ifdef LSM6DSO32_DEBUG
    Serial.println("spi_write()");
#endif
    SPIClass *spi = (SPIClass *)spiPtr;
    
    // Begin transaction
    spi->beginTransaction(SPISettings(DEFAULT_SPI_FREQ, MSBFIRST, SPI_MODE0));
    
    // Assert chip select
    digitalWrite(chipSelect, LOW);
    
    // Write register address
    spi->transfer(regAddr);
    
    // Write the register data
    for (size_t index = 0; index < len; index++) {
        spi->transfer(regData[index]);
    }
    
    // Deassert chip select
    digitalWrite(chipSelect, HIGH);
    
    // End transaction
    spi->endTransaction();
    
    return 0;
}
