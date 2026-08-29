/*
 * comms.c
 *
 * Comms Module: sensors (IMU), PuTTY serial command parsing, button
 * reading, and alarm/status LED control. Relocated from main.c without
 * behavior changes.
 */

#include "comms.h"
#include "bno055_dfrobot.h"
#include <stdlib.h>

static BNO055_t bno;
static BNO055_Euler_t bno_euler;

static char serial_rx_buffer[64];
static uint8_t serial_rx_index = 0;

void Comms_Init(I2C_HandleTypeDef *hi2c) {
  BNO055_Attach(&bno, hi2c, &hcom_uart[COM1],
                BNO055_I2C_ADDRESS_7BIT_DEFAULT);

  BNO055_Status_t status = BNO055_Begin(&bno);

  if (status != BNO055_STATUS_OK) {
    BNO055_PrintStatus(&bno, status);
  }
}

void Comms_SendWelcomeMessage(void) {
  char welcome_msg[] =
      "\r\n"
      "=== Stewart Platform Control ===\r\n"
      "Enter command as:\r\n"
      "Z,roll,pitch\r\n"
      "\r\n"
      "Example:\r\n"
      "10,5,-3\r\n"
      "\r\n"
      "Z     = heave in mm\r\n"
      "roll  = angle in degrees\r\n"
      "pitch = angle in degrees\r\n"
      "\r\n"
      "Type command and press ENTER:\r\n> ";

  HAL_UART_Transmit(
      &hcom_uart[COM1],
      (uint8_t *)welcome_msg,
      sizeof(welcome_msg) - 1,
      HAL_MAX_DELAY
  );
}

int Comms_ReadIMU(double *roll, double *pitch) {
  if (BNO055_ReadEuler(&bno, &bno_euler) != BNO055_STATUS_OK) {
    return 0;
  }

  BNO055_PrintEuler(&bno, &bno_euler);

  *roll = bno_euler.roll;
  *pitch = bno_euler.pitch;

  return 1;
}

int ParsePoseCommand(char *text, double *Z_cmd, double *roll_cmd,
                     double *pitch_cmd) {
  char *end;

  /*
   * Read Z
   */
  double z = strtod(text, &end);

  if (end == text || *end != ',') {
    return 0;
  }

  /*
   * Read roll
   */
  double r = strtod(end + 1, &end);

  if (*end != ',') {
    return 0;
  }

  /*
   * Read pitch
   */
  double p = strtod(end + 1, &end);

  /*
   * Check that nothing invalid remains.
   */
  while (*end == ' ') {
    end++;
  }

  if (*end != '\0') {
    return 0;
  }

  /*
   * Command is valid.
   */
  *Z_cmd = z;
  *roll_cmd = r;
  *pitch_cmd = p;

  return 1;
}

int Serial_ReadPoseCommand(double *Z_cmd, double *roll_cmd, double *pitch_cmd) {
  uint8_t rx_byte;

  /*
   * Check whether one byte has arrived.
   *
   * 1 ms timeout means this does not block the program
   * for very long.
   */
  if (HAL_UART_Receive(&hcom_uart[COM1], &rx_byte, 1, 1) != HAL_OK) {
    return 0;
  }

  /*
   * ENTER received.
   *
   * Accept both CR and LF because different terminals
   * use different line endings.
   */
  if ((rx_byte == '\r') || (rx_byte == '\n')) {
    /*
     * Ignore empty line.
     *
     * This is useful when the PC sends CR+LF.
     */
    if (serial_rx_index == 0) {
      return 0;
    }

    /*
     * Terminate C string.
     */
    serial_rx_buffer[serial_rx_index] = '\0';

    serial_rx_index = 0;

    /*
     * Convert ASCII command into numbers.
     */
    return ParsePoseCommand(serial_rx_buffer, Z_cmd, roll_cmd, pitch_cmd);
  }

  /*
   * Store received character.
   */
  if (serial_rx_index < (sizeof(serial_rx_buffer) - 1)) {
    serial_rx_buffer[serial_rx_index] = (char)rx_byte;

    serial_rx_index++;
  } else {
    /*
     * Buffer overflow.
     * Throw away command and start again.
     */
    serial_rx_index = 0;
  }

  return 0;
}

GPIO_PinState Comms_ReadButton(void) {
  return HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);
}

void Comms_SetAlarmLED(GPIO_PinState state) {
  if (state == GPIO_PIN_SET) {
    BSP_LED_On(LED_GREEN);
  } else {
    BSP_LED_Off(LED_GREEN);
  }
}

void Comms_UpdateAlarmLED(void) {
  GPIO_PinState buttonState = Comms_ReadButton();

  if (buttonState == GPIO_PIN_SET) {
    /* Button pressed */
    Comms_SetAlarmLED(GPIO_PIN_SET);
  } else {
    /* Button released */
    Comms_SetAlarmLED(GPIO_PIN_RESET);
  }
}
