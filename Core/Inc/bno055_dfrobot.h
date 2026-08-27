/*
 * bno055_dfrobot.h
 *
 * Pure-C STM32 HAL port of the BNO055 portion of the DFRobot Gravity
 * 10-DOF IMU AHRS example/library.
 *
 * Designed for STM32CubeMX / STM32CubeIDE projects.
 * No Arduino.h, Wire.h, C++ classes, or DFRobot C++ library required.
 */

#ifndef BNO055_DFROBOT_H
#define BNO055_DFROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define BNO055_I2C_ADDRESS_7BIT_DEFAULT  0x28U

typedef enum
{
    BNO055_STATUS_OK = 0,
    BNO055_STATUS_ERROR,
    BNO055_STATUS_DEVICE_NOT_DETECTED,
    BNO055_STATUS_READY_TIMEOUT,
    BNO055_STATUS_DEVICE_STATUS_ERROR,
    BNO055_STATUS_PARAMETER_ERROR
} BNO055_Status_t;

typedef struct
{
    float heading;   /* yaw / heading, degrees */
    float roll;      /* degrees */
    float pitch;     /* degrees */
} BNO055_Euler_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} BNO055_Quaternion_t;

typedef struct
{
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} BNO055_Calibration_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    UART_HandleTypeDef *huart;   /* optional debug UART; may be NULL */

    uint16_t address_hal;        /* HAL address = 7-bit address << 1 */
    uint8_t current_page;

    /* Startup/re-zero reference in absolute BNO055 Euler degrees. */
    float zero_heading;
    float zero_roll;
    float zero_pitch;
    bool zero_valid;

    BNO055_Status_t last_status;
} BNO055_t;

/*
 * Attach the CubeMX-generated handles to the driver.
 *
 * address_7bit:
 *   normally 0x28, matching the DFRobot sample.
 *
 * huart:
 *   may be NULL if no debug text is needed.
 */
void BNO055_Attach(BNO055_t *dev,
                   I2C_HandleTypeDef *hi2c,
                   UART_HandleTypeDef *huart,
                   uint8_t address_7bit);

/*
 * Equivalent to the important initialization performed by the supplied
 * DFRobot begin():
 *
 *   verify CHIP_ID
 *   reset
 *   wait for device
 *   CONFIG mode
 *   axis map P1
 *   degree units
 *   accelerometer +/-4 g
 *   gyroscope +/-2000 dps
 *   normal power
 *   NDOF fusion mode
 *   wait until BNO055 gyro calibration reaches level 3
 *   average 32 samples as yaw=0, pitch=0, roll=0
 */
BNO055_Status_t BNO055_Begin(BNO055_t *dev);

/* Software-reset the BNO055 and wait for it to restart. */
BNO055_Status_t BNO055_Reset(BNO055_t *dev);

/*
 * Read zero-relative fused Euler orientation.
 *
 * BNO055_Begin() waits until gyro calibration reaches level 3, then
 * averages 32 startup readings to define the zero orientation.
 * Heading/yaw is wrapped to -180..+180 degrees.
 */
BNO055_Status_t BNO055_ReadEuler(BNO055_t *dev,
                                 BNO055_Euler_t *euler);

/* Read the original absolute fused Euler values directly from the BNO055. */
BNO055_Status_t BNO055_ReadEulerAbsolute(BNO055_t *dev,
                                         BNO055_Euler_t *euler);

/*
 * Make the current physical orientation the new (0,0,0) reference by
 * averaging 32 readings. This takes about 620 ms.
 */
BNO055_Status_t BNO055_SetCurrentOrientationAsZero(BNO055_t *dev);

/* Read fused quaternion from the BNO055. */
BNO055_Status_t BNO055_ReadQuaternion(BNO055_t *dev,
                                      BNO055_Quaternion_t *q);

/* Read system / gyro / accel / magnetometer calibration levels (0..3). */
BNO055_Status_t BNO055_ReadCalibration(BNO055_t *dev,
                                       BNO055_Calibration_t *cal);

/* Verify CHIP_ID == 0xA0. */
bool BNO055_TestConnection(BNO055_t *dev);

/*
 * Optional UART helpers.
 *
 * BNO055_PrintEuler() uses a fixed Simulink-friendly serial frame:
 *
 *   ypr<TAB>yaw[8]<TAB>pitch[8]<TAB>roll[8]<CR><LF>
 *
 * The payload after the header is exactly 26 bytes.
 */
void BNO055_PrintEuler(BNO055_t *dev, const BNO055_Euler_t *euler);

void BNO055_PrintStatus(BNO055_t *dev, BNO055_Status_t status);

const char *BNO055_StatusString(BNO055_Status_t status);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_DFROBOT_H */
