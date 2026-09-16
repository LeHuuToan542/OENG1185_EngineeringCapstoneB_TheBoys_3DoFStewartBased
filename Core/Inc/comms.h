/*
 * comms.h
 *
 * Comms Module: sensors (IMU), PuTTY serial command parsing, button
 * reading, and alarm/status LED control.
 */

#ifndef COMMS_H
#define COMMS_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Attach the IMU driver and run its startup/calibration sequence. */
void Comms_Init(I2C_HandleTypeDef *hi2c);

/* Send a NUL-terminated string over the PuTTY serial link. */
void Comms_Print(const char *text);

/* Print the Stewart platform welcome banner over the PuTTY serial link. */
void Comms_SendWelcomeMessage(void);

/*
 * Read the fused IMU orientation. Returns 1 and writes *roll and *pitch
 * (degrees) on success, 0 otherwise.
 */
int Comms_ReadIMU(double *roll, double *pitch);

/*
 * Read one line typed over PuTTY, if a complete line has arrived.
 *
 *   2 -> the literal command "IMU" (switch to IMU mode)
 *   1 -> a "Z,roll,pitch" command; *Z_cmd, *roll_cmd, *pitch_cmd written
 *   3 -> an "An,mm,dir" single-actuator test command (e.g. "A1,10,1");
 *        *test_actuator (0-based), *test_dir (1 = extend, 0 = retract) and
 *        *test_mm (jog distance in mm) written
 *   4 -> the literal command "LIM" (watch the limit switches live)
 *   0 -> nothing to report yet (no complete line has arrived)
 *  -1 -> a complete line arrived but could not be understood
 */
int Serial_ReadPoseCommand(double *Z_cmd, double *roll_cmd, double *pitch_cmd,
                           int *test_actuator, int *test_dir,
                           double *test_mm);

/* Read the start/test push button. */
GPIO_PinState Comms_ReadButton(void);

/* Drive the alarm/status LED. */
void Comms_SetAlarmLED(GPIO_PinState state);

/* Mirror the start/test button state onto the alarm/status LED. */
void Comms_UpdateAlarmLED(void);

void UART_SendLegLengths(double q[3]);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_H */
