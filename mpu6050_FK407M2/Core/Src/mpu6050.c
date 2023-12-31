// Include 
#include "main.h"
#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "mpu6050.h"

/**
 * @brief 
 * Calibration Offset For MPU6050
 */
float Accel_X_offset = 0;
float Accel_Y_offset = 0;
float Accel_Z_offset = 0;
float Gyro_X_offset  = 0;
float Gyro_Y_offset  = 0;
float Gyro_Z_offset  = 0;
float Temp_offset    = 0;
MPU6050_RAW_t raw_offset;

/**
 * @brief 
 * Calculate from raw value to Roll, Yaw, And Pitch
 */
float Accel_AngleX, Accel_AngleY;
float Gyro_AngleX, Gyro_AngleY;

uint64_t prev_time, current_time;
uint32_t elapsed_time; 

void mpu6050_calculate_angle(MPU6050_RAW_t * Raw, MPU6050_ANGLE_t * angle)
{
    // Calculating Roll and Pitch from the accelerometer data
    Accel_AngleX = (atan(Raw->Accel_Y / sqrt(pow(Raw->Accel_X, 2) + pow(Raw->Accel_Z, 2))) * 180 / 3.14F); 
    Accel_AngleY = (atan(-1 * Raw->Accel_X / sqrt(pow(Raw->Accel_Y, 2) + pow(Raw->Accel_Z, 2))) * 180 / 3.14F);

    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    Gyro_AngleX = Gyro_AngleX + Raw->Gyro_X * elapsed_time; // deg/s * s = deg
    Gyro_AngleY = Gyro_AngleY + Raw->Gyro_Y * elapsed_time;
    angle->yaw  = angle->yaw + Raw->Gyro_Z * elapsed_time;

    // Complementary filter - combine acceleromter and gyro angle values
    angle->roll     = 0.96 * Gyro_AngleX + 0.04 * Accel_AngleX;
    angle->pitch    = 0.96 * Gyro_AngleY + 0.04 * Accel_AngleY;
}

char mpu6050_init(void)
{
    uint8_t check;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_WHO_AM_I, 1, &check, 1, 1000);

    if (check == 0x68)  // Get MPU6050 Echo
    {
        // Wakeup MPU6050, Use Internal 8MHz OSC
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_PWR_MGMT_1, 1, (uint8_t *)0x00, 1, 1000);
        // Set Sample Rate = 1KHz (0x07)
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_PWR_MGMT_1, 1, (uint8_t *)0x07, 1, 1000);
        // Set Accel Full Scale Range = +/- 2g (0x00)
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_ACCEL_CONFIG, 1, (uint8_t *)0x00, 1, 1000);
        // Set Gyro Full Scale Range = +/- 250deg/s
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_GYRO_CONFIG, 1, (uint8_t *)0x00, 1, 1000);

        return 1;   // Config Complete
    }
    return 0;   // Config Failure
}

char mpu6050_accel_read(MPU6050_RAW_t * Raw)
{
    uint8_t status = 0x00;
    uint8_t read_data[6] = {0x00};
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, read_data, 6, 1000);

    if (status == HAL_OK)
    {
        // Read Raw Value
        int16_t Accel_X_Raw = (int16_t)(read_data[0] << 8 | read_data[1]);
        int16_t Accel_Y_Raw = (int16_t)(read_data[2] << 8 | read_data[3]);
        int16_t Accel_Z_Raw = (int16_t)(read_data[4] << 8 | read_data[5]);
    
        /**
         * @brief 
         * Convert To Accel (m/2), Divide By 16384 From +/- 2g
         */
        Raw->Accel_X = (Accel_X_Raw / 16384.0) - Accel_X_offset;
        Raw->Accel_Y = (Accel_Y_Raw / 16384.0) - Accel_Y_offset;
        Raw->Accel_Z = (Accel_Z_Raw / 16384.0) - Accel_Z_offset;

        return 1;   // Read Complete

        prev_time = current_time;
    }

    return 0;   // Read Failure
}

char mpu6050_gyro_read(MPU6050_RAW_t * Raw)
{
    current_time = HAL_GetTick(); 
    elapsed_time = (current_time - prev_time) / 1000; // Divide by 1000 to get seconds
    
    uint8_t status = 0x00;
    uint8_t read_data[6] = {0x00};
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_GYRO_XOUT_H, 1, read_data, 6, 1000);

    if (status == HAL_OK)
    {
        // Read Raw Value
        int16_t Gyro_X_Raw = (int16_t)(read_data[0] << 8 | read_data[1]);
        int16_t Gyro_Y_Raw = (int16_t)(read_data[2] << 8 | read_data[3]);
        int16_t Gyro_Z_Raw = (int16_t)(read_data[4] << 8 | read_data[5]);
    
        /**
         * @brief 
         * Convert To Gyro (Deg), Divide By 131 From +/- 250 Deg/s
         */
        Raw->Gyro_X = (Gyro_X_Raw / 131.0) - Gyro_X_offset;
        Raw->Gyro_Y = (Gyro_Y_Raw / 131.0) - Gyro_Y_offset;
        Raw->Gyro_Z = (Gyro_Z_Raw / 131.0) - Gyro_Z_offset;
    
        return 1;  // Read Complete
    }

    return 0;   // Read Failure
}

char mpu6050_calibreation(void)
{
    for (uint16_t i = 0; i < calibration_steps; i++)
    {
        mpu6050_accel_read(&raw_offset);
        mpu6050_gyro_read(&raw_offset);

        Accel_X_offset  += raw_offset.Accel_X;
        Accel_Y_offset  += raw_offset.Accel_Y;
        Accel_Z_offset  += raw_offset.Accel_Z;
        Gyro_X_offset   += raw_offset.Gyro_X;
        Gyro_Y_offset   += raw_offset.Gyro_Y;
        Gyro_Z_offset   += raw_offset.Gyro_Z;

        HAL_Delay(20);
    }

    Accel_X_offset /= 100;  Gyro_X_offset /= 100;
    Accel_Y_offset /= 100;  Gyro_Y_offset /= 100;
    Accel_Z_offset /= 100;  Gyro_Z_offset /= 100;

    printf("Offset: %.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
            Accel_X_offset, Accel_Y_offset, Accel_Z_offset,
            Gyro_X_offset, Gyro_Y_offset, Gyro_Z_offset);

    if (Accel_X_offset != 0)
        return 1;

    return 0;
}