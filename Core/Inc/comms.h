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

/* Print the Stewart platform welcome banner over the PuTTY serial link. */
void Comms_SendWelcomeMessage(void);

/*
 * Read the fused IMU orientation. Returns 1 and writes *roll and *pitch
 * (degrees) on success, 0 otherwise.
 */
int Comms_ReadIMU(double *roll, double *pitch);

/*
 * Parse a "Z,roll,pitch" command received over PuTTY.
 * Returns 1 and writes *Z_cmd, *roll_cmd, and *pitch_cmd on a complete,
 * valid command, 0 otherwise (including while a command is still being
 * typed).
 */
int Serial_ReadPoseCommand(double *Z_cmd, double *roll_cmd, double *pitch_cmd);

/* Read the start/test push button. */
GPIO_PinState Comms_ReadButton(void);

/* Drive the alarm/status LED. */
void Comms_SetAlarmLED(GPIO_PinState state);

/* Mirror the start/test button state onto the alarm/status LED. */
void Comms_UpdateAlarmLED(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_H */
