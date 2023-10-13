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

#ifndef _ADAFRUIT_LSM6DSO32_H
#define _ADAFRUIT_LSM6DSO32_H

#include <SPI.h>
#include "lsm6dso32-pid-master/lsm6dso32_reg.h" //LSM6DSO32 API functions

// Constants
//#define LSM6DSO32_DEBUG   // Uncomment to allow debug messages to console
#define DEFAULT_SPI_FREQ 1000000
#define LSM6DSO32_CHIP_ID 0x6C // LSM6DSO32 default device id from WHOAMI
#define G_TO_METER_PER_SEC_SQUARED 9.80665F
#define METER_PER_SEC_SQUARED_TO_G 0.101971F
#define DEGREES_TO_RADIANS 0.017453293F
#define RADIANS_TO_DEGREES 57.29577793F

// Error codes
#define LSM6DSO32_ERROR_INVALID_INPUT_PARAMETERS -1

typedef enum accel_fullscale_range {
    LSM6DSO32_RANGE_4G  = 0x00,
    LSM6DSO32_RANGE_8G  = 0x02,
    LSM6DSO32_RANGE_16G = 0x03,
    LSM6DSO32_RANGE_32G = 0x01,
} lsm6dso32_accel_range_t;

typedef enum accel_data_rate {
    LSM6DSO32_ACCEL_SHUTDOWN,
    LSM6DSO32_ACCEL_6_5_HZ,
    LSM6DSO32_ACCEL_12_5_HZ,
    LSM6DSO32_ACCEL_26_HZ,
    LSM6DSO32_ACCEL_52_HZ,
    LSM6DSO32_ACCEL_104_HZ,
    LSM6DSO32_ACCEL_208_HZ,
    LSM6DSO32_ACCEL_417_HZ,
    LSM6DSO32_ACCEL_833_HZ,
    LSM6DSO32_ACCEL_1667_HZ,
    LSM6DSO32_ACCEL_3333_HZ,
    LSM6DSO32_ACCEL_6667_HZ,
} lsm6dso32_accel_data_rate_t;

typedef enum gyro_fullscale_range {
    LSM6DSO32_RANGE_250DPS  = 0,
    LSM6DSO32_RANGE_125DPS  = 1,
    LSM6DSO32_RANGE_500DPS  = 2,
    LSM6DSO32_RANGE_1000DPS = 4,
    LSM6DSO32_RANGE_2000DPS = 6
} lsm6dso32_gyro_range_t;

typedef enum gyro_data_rate {
    LSM6DSO32_GYRO_SHUTDOWN,
    LSM6DSO32_GYRO_12_5_HZ,
    LSM6DSO32_GYRO_26_HZ,
    LSM6DSO32_GYRO_52_HZ,
    LSM6DSO32_GYRO_104_HZ,
    LSM6DSO32_GYRO_208_HZ,
    LSM6DSO32_GYRO_417_HZ,
    LSM6DSO32_GYRO_833_HZ,
    LSM6DSO32_GYRO_1667_HZ,
    LSM6DSO32_GYRO_3333_HZ,
    LSM6DSO32_GYRO_6667_HZ,
} lsm6dso32_gyro_data_rate_t;

typedef enum accel_power_mode {
    LSM6DSO32_ACCEL_POWER_DOWN,
    LSM6DSO32_ACCEL_ULTRA_LOW_POWER,
    LSM6DSO32_ACCEL_LOW_POWER,
    LSM6DSO32_ACCEL_NORMAL,
    LSM6DSO32_ACCEL_HIGH_PERFORMANCE,
} lsm6dso32_accel_power_mode_t;

typedef enum gyro_power_mode {
    LSM6DSO32_GYRO_POWER_DOWN,
    LSM6DSO32_GYRO_LOW_POWER,
    LSM6DSO32_GYRO_NORMAL,
    LSM6DSO32_GYRO_HIGH_PERFORMANCE,
} lsm6dso32_gyro_power_mode_t;

typedef enum accel_lpf2_bandwidth {
    LSM6DSO32_ACCEL_LPF2_DIV4,
    LSM6DSO32_ACCEL_LPF2_DIV10,
    LSM6DSO32_ACCEL_LPF2_DIV20,
    LSM6DSO32_ACCEL_LPF2_DIV45,
    LSM6DSO32_ACCEL_LPF2_DIV100,
    LSM6DSO32_ACCEL_LPF2_DIV200,
    LSM6DSO32_ACCEL_LPF2_DIV400,
    LSM6DSO32_ACCEL_LPF2_DIV800
} lsm6dso32_accel_lpf2_bw_t;

typedef enum accel_hpf_bandwidth {
    LSM6DSO32_ACCEL_HPF_DIV4,
    LSM6DSO32_ACCEL_HPF_DIV10,
    LSM6DSO32_ACCEL_HPF_DIV20,
    LSM6DSO32_ACCEL_HPF_DIV45,
    LSM6DSO32_ACCEL_HPF_DIV100,
    LSM6DSO32_ACCEL_HPF_DIV200,
    LSM6DSO32_ACCEL_HPF_DIV400,
    LSM6DSO32_ACCEL_HPF_DIV800
} lsm6dso32_accel_hpf_bw_t;

// See Table 60 in data sheet for actual bandwidth
// values which depend on the chosed odr.
typedef enum gyro_lpf_bandwidth {
    LSM6DSO32_GYRO_LPF_LVL0,
    LSM6DSO32_GYRO_LPF_LVL1,
    LSM6DSO32_GYRO_LPF_LVL2,
    LSM6DSO32_GYRO_LPF_LVL3,
    LSM6DSO32_GYRO_LPF_LVL4,
    LSM6DSO32_GYRO_LPF_LVL5,
    LSM6DSO32_GYRO_LPF_LVL6,
    LSM6DSO32_GYRO_LPF_LVL7,
}lsm6dso32_gyro_lpf_bw_t;

typedef enum gyro_hpf_bandwidth {
    LSM6DSO32_GYRO_HPF_0_016HZ  = 0x80,
    LSM6DSO32_GYRO_HPF_0_065HZ  = 0x81,
    LSM6DSO32_GYRO_HPF_0_260HZ  = 0x82,
    LSM6DSO32_GYRO_HPF_1_04HZ   = 0x83
}lsm6dso32_gyro_hpf_bw_t;

typedef enum accel_offset_correction_weight {
    LSM6DSO32_ACCEL_OFFSET_WEIGHT_1MG,
    LSM6DSO32_ACCEL_OFFSET_WEIGHT_16MG,
}lsm6dso32_accel_offset_corr_t;


//==============================================================================================
// Class LSM6DSO32
//==============================================================================================
// A class that wraps the LSM6DSO32 API functions provided by STMicroelectronics.
//==============================================================================================

class LSM6DSO32 {
public:
    // Constrluctor
    LSM6DSO32();
    
    // Initialization
    bool begin(uint8_t chipSelect, SPIClass *spiPtr = &SPI, uint32_t spiFrequency = DEFAULT_SPI_FREQ);
    
    // Accelerometer sensor configuration
    void setAccelRange(lsm6dso32_accel_range_t range);
    lsm6dso32_accel_range_t getAccelRange(void);
    int8_t setAccelDataRate(lsm6dso32_accel_data_rate_t odr, lsm6dso32_accel_power_mode_t powerMode = LSM6DSO32_ACCEL_HIGH_PERFORMANCE);
    lsm6dso32_accel_data_rate_t getAccelDataRate(void);
    void enableAccelLowPassFilter2(bool enable, lsm6dso32_accel_lpf2_bw_t bandwidth);
    void enableAccelHighPassFilter(bool enable, lsm6dso32_accel_hpf_bw_t bandwidth,
                                   bool refModeEnable = false);
    void enableAccelOffsetCorrection(bool enable, lsm6dso32_accel_offset_corr_t weight,
                                     int8_t xAxisCorrection = 0, int8_t yAxisCorrection = 0,
                                     int8_t zAxisCorrection = 0);
    
    // Gyro Sensor configuration
    void setGyroRange(lsm6dso32_gyro_range_t range);
    lsm6dso32_gyro_range_t getGyroRange(void);
    int8_t setGyroDataRate(lsm6dso32_gyro_data_rate_t odr, lsm6dso32_gyro_power_mode_t powerMode = LSM6DSO32_GYRO_HIGH_PERFORMANCE);
    lsm6dso32_gyro_data_rate_t getGyroDataRate(void);
    void enableGyroLowPassFilter(bool enable, lsm6dso32_gyro_lpf_bw_t bandwidth);
    void enableGyroHighPassFilter(bool enable, lsm6dso32_gyro_hpf_bw_t bandwidth);
    
    // Read sensor data
    void readSensorData(void);
    void readAccelData(void);
    void readGyroData(void);
    void readTempData(void);
    
    // Get the most recent sensor data values
    float getTemperature(void);
    float getXAxisAccel(void);
    float getYAxisAccel(void);
    float getZAxisAccel(void);
    float getXAxisRate(void);
    float getYAxisRate(void);
    float getZAxisRate(void);
    
private:
    bool _init();
    void _reset(void);
    
    float temperature;      // Most recent temperature reading in degrees C
    float xAxisAccel;       // Most recent X-Axis accelerometer reading in G's
    float yAxisAccel;       // Most recent Y-Axis accelerometer reading in G's
    float zAxisAccel;       // Most recent Z-Axis accelerometer reading in G's
    float xAxisRate;       // Most recent X-Axis gyro reading degrees per second
    float yAxisRate;       // Most recent Y-Axis gyro reading degrees per second
    float zAxisRate;       // Most recent Z-Axis gyro reading degrees per second
    lsm6dso32_accel_range_t accelRange; // Current accelerometer fullscale range setting
    lsm6dso32_gyro_range_t gyroRange;   // Current gyro fullscale range setting
    float rateScaleFactor;
    float accelerationScaleFactor;
    stmdev_ctx_t sensorAPI_intf; //Structure holds pointers to spi read/write functions
                                 //and spi bus object
};

#endif
