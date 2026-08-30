/*
 * control.c
 *
 * Control Module. Relocated from main.c's while(1) loop without behavior
 * changes: same IMU-active/PuTTY-commented decision logic, same call order.
 */

#include "control.h"
#include "comms.h"
#include "ik.h"
#include "drive.h"

double q[3];

double Z = 50;
double roll = 0;
double pitch = 0;

void Control_Update(void) {
  double Z_cmd, roll_cmd, pitch_cmd;

  //IMU CONTROL - UNCOMMENT TO USE IMU FOR CONTROL
  if (Comms_ReadIMU(&roll, &pitch)) {
    simscape_ik(Z, roll, pitch, q);
    MoveActuatorsToTarget(q);
  }

  //PuTTy CONTROL - UNCOMMENT TO USE PuTTy FOR CONTROL
  // if (Serial_ReadPoseCommand(&Z_cmd, &roll_cmd, &pitch_cmd)) {
  //   /*
  //    * Save new desired platform pose.
  //    */
  //   Z = Z_cmd;
  //   roll = roll_cmd;
  //   pitch = pitch_cmd;
  //   char msg[] = "POSE COMMAND RECEIVED\r\n";
  //   HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
  //   simscape_ik(Z, roll, pitch, q);
  //   MoveActuatorsToTarget(q);
  //   char prompt[] = "\r\nEnter next command:\r\n> ";
  //   HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)prompt, sizeof(prompt) - 1, HAL_MAX_DELAY);
  // }
}
