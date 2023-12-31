#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"


/**
 * @brief 
 * Measurement Raw Value
 */
typedef struct MPU6050_Raw_t
{
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    float Temp;
} MPU6050_RAW_t;

typedef struct MPU6050_Angle_t
{
    float roll;
    float yaw;
    float pitch;
} MPU6050_ANGLE_t;

#define calibration_steps   200

/**
 * @brief 
 * It is the device address for the MPU6050 module
 */
#define MPU6050_ADDR        0xD0

/**
 * @brief 
 * This register is used to verify the identity of the device. The contents
 * of WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C
 * address. The least significant bit of the MPU-60X0’s I2C address is
 * determined by the value of the AD0 pin. The value of the AD0 pin is
 * not reflected in this register
 */
#define REG_WHO_AM_I        0x75

/**
 * @brief 
 * This register allows the user to configure the power mode and clock
 * source. It also provides a bit for resetting the entire device, and a bit
 * for disabling the temperature sensor
 */
#define REG_PWR_MGMT_1      0x6B

/**
 * @brief 
 * This register specifies the divider from the gyroscope output rate
 * used to generate the Sample Rate for the MPU-6050.
 */
#define REG_SMPLRT_DIV      0x19

/**
 * @brief 
 * This register is used to trigger gyroscope self-test and configure the
 * gyroscopes’ full scale range.
 */
#define REG_GYRO_CONFIG     0x1B

/**
 * @brief 
 * This register is used to trigger the accelerometer self test and
 * configure the accelerometer full scale range. This register also
 * configures the Digital High Pass Filter (DHPF).
 */
#define REG_ACCEL_CONFIG    0x1C

/**
 * @brief 
 * These registers store the most recent accelerometer measurements.
 * From Accel_X Start. (Big-endian)
 */
#define REG_ACCEL_XOUT_H    0x3B

/**
 * @brief 
 * These registers store the most recent gyroscope measurements.
 * From Gyro_X Start. (Big-endian)
 */
#define REG_GYRO_XOUT_H     0x43

extern char mpu6050_init(void);
extern char mpu6050_calibreation(void);
extern char mpu6050_accel_read(MPU6050_RAW_t *); 
extern char mpu6050_gyro_read(MPU6050_RAW_t *); 
extern void mpu6050_calculate_angle(MPU6050_RAW_t *, MPU6050_ANGLE_t *);

#endif // ! __MPU6050_H