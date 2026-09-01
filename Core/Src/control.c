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

/*
 * Mutually exclusive control source, switched over PuTTY: typing "IMU"
 * enables IMU mirror mode, and any "Z,roll,pitch" command switches back
 * to PuTTY mode. Starts in PuTTY mode so nothing moves until commanded.
 */
static int imu_mode = 0;

void Control_Update(void) {
  double Z_cmd, roll_cmd, pitch_cmd;

  int cmd = Serial_ReadPoseCommand(&Z_cmd, &roll_cmd, &pitch_cmd);

  if (cmd == 2) {
    imu_mode = 1;
    char msg[] = "\r\nIMU MODE ENABLED\r\n> ";
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
  } else if (cmd == 1) {
    imu_mode = 0;
    /*
     * Save new desired platform pose.
     */
    Z = Z_cmd;
    roll = roll_cmd;
    pitch = pitch_cmd;
    char msg[] = "POSE COMMAND RECEIVED\r\n";
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    simscape_ik(Z, roll, pitch, q);
    MoveActuatorsToTarget(q);
    char prompt[] = "\r\nEnter next command:\r\n> ";
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)prompt, sizeof(prompt) - 1, HAL_MAX_DELAY);
  }

  if (imu_mode) {
    if (Comms_ReadIMU(&roll, &pitch)) {
      simscape_ik(Z, roll, pitch, q);
      MoveActuatorsToTarget(q);
    }
  }
}
