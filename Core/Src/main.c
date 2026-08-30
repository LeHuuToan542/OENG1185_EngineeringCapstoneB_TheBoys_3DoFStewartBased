/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055_dfrobot.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979323846
#define MOTOR_STEPS_PER_REV 200.0
#define MICROSTEP 1
#define MM_PER_REV 12.0

#define STEPS_PER_MM ((MOTOR_STEPS_PER_REV * MICROSTEP) / MM_PER_REV)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
StepperMotor FrontActuator = {.STEP_Port = STEP_OUT_ACT1_GPIO_Port,
                          .STEP_Pin = STEP_OUT_ACT1_Pin,

                          .DIR_Port = DIR_OUT_ACT1_GPIO_Port,
                          .DIR_Pin = DIR_OUT_ACT1_Pin,

                          .current_stroke_length_mm = 0.0}; // Initialize current stroke length to 0.0 mm

StepperMotor BackRightActuator = {.STEP_Port = STEP_OUT_ACT2_GPIO_Port,
                          .STEP_Pin = STEP_OUT_ACT2_Pin,

                          .DIR_Port = DIR_OUT_ACT2_GPIO_Port,
                          .DIR_Pin = DIR_OUT_ACT2_Pin,
                          .current_stroke_length_mm = 0.0}; // Initialize current stroke length to 0.0 mm

StepperMotor BackLeftActuator = {.STEP_Port = STEP_OUT_ACT3_GPIO_Port,
                          .STEP_Pin = STEP_OUT_ACT3_Pin,

                          .DIR_Port = DIR_OUT_ACT3_GPIO_Port,
                          .DIR_Pin = DIR_OUT_ACT3_Pin,
                          .current_stroke_length_mm = 0.0}; // Initialize current stroke length to 0.0 mm

double q[3];

double Z = 50;
double roll = 0;
double pitch = 0;

static BNO055_t bno;
static BNO055_Euler_t bno_euler;

static char serial_rx_buffer[64];
static uint8_t serial_rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// void Stepper_Move_Select(StepperMotor *motor, GPIO_PinState direction,
//                   uint32_t pulses) {
//   HAL_GPIO_WritePin(motor->DIR_Port, motor->DIR_Pin, direction);
//   HAL_Delay(1);
//   for (uint32_t i = 0; i < pulses; i++) {
//     HAL_GPIO_WritePin(motor->STEP_Port, motor->STEP_Pin, GPIO_PIN_SET);
//     HAL_Delay(1);
//     HAL_GPIO_WritePin(motor->STEP_Port, motor->STEP_Pin, GPIO_PIN_RESET);
//     HAL_Delay(1);
//   }
// }

void simscape_ik(double Z, double roll, double pitch, double q[3]) {
  /* ================================================================
   * 1. Geometry
   * ================================================================ */

  const double r = 100.0;             // Platform/base radius [mm]
  const double stroke_length = 200.0; // Actuator stroke specification [mm]

  const double L_resting = 125.0 + stroke_length;
  const double stroke_home = 0.0;

  /* ================================================================
   * 2. Input clamping
   * ================================================================ */

  const double max_tilt_deg = 25.0;

  double total_tilt = sqrt((roll * roll) + (pitch * pitch));

  /*
   * Limit combined roll/pitch vector magnitude to 25 degrees.
   */
  if ((total_tilt > max_tilt_deg) && (total_tilt > 0.0)) {
    double scale = max_tilt_deg / total_tilt;

    roll *= scale;
    pitch *= scale;
  }

  /*
   * Limit Z to +/-40 mm.
   */
  double z_rel_clamped = Z;

  if (z_rel_clamped > 40.0) {
    z_rel_clamped = 40.0;
  } else if (z_rel_clamped < -40.0) {
    z_rel_clamped = -40.0;
  }

  /*
   * Convert degrees to radians.
   */
  double rad_roll = roll * (PI / 180.0);
  double rad_pitch = pitch * (PI / 180.0);

  /* ================================================================
   * 3. Orientation matrices
   *
   * R_sb = Ry * Rx
   * ================================================================ */

  double Rx[3][3] = {{1.0, 0.0, 0.0},

                     {0.0, cos(rad_roll), -sin(rad_roll)},

                     {0.0, sin(rad_roll), cos(rad_roll)}};

  double Ry[3][3] = {{cos(rad_pitch), 0.0, sin(rad_pitch)},

                     {0.0, 1.0, 0.0},

                     {-sin(rad_pitch), 0.0, cos(rad_pitch)}};

  /*
   * R_sb = Ry * Rx
   */
  double R_sb[3][3];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R_sb[i][j] = 0.0;

      for (int k = 0; k < 3; k++) {
        R_sb[i][j] += Ry[i][k] * Rx[k][j];
      }
    }
  }

  /* ================================================================
   * 4. Joint positions
   *
   * Joint arrangement:
   *
   *        actuator 1
   *            0 deg
   *
   *     actuator 3       actuator 2
   *        240 deg          120 deg
   *
   * ================================================================ */

  double angles_deg[3] = {0.0, 120.0, 240.0};

  double a[3][3];
  double b_local[3][3];

  for (int i = 0; i < 3; i++) {
    double angle_rad = angles_deg[i] * (PI / 180.0);

    /*
     * Each column represents one joint:
     *
     * a[0][i] = X
     * a[1][i] = Y
     * a[2][i] = Z
     */
    a[0][i] = r * cos(angle_rad);
    a[1][i] = r * sin(angle_rad);
    a[2][i] = 0.0;

    /*
     * MATLAB:
     *
     * b_local = a;
     */
    b_local[0][i] = a[0][i];
    b_local[1][i] = a[1][i];
    b_local[2][i] = a[2][i];
  }

  /* ================================================================
   * Rotate platform joint vectors:
   *
   * u = R_sb * b_local
   * ================================================================ */

  double u[3][3];

  for (int joint = 0; joint < 3; joint++) {
    for (int row = 0; row < 3; row++) {
      u[row][joint] = 0.0;

      for (int k = 0; k < 3; k++) {
        u[row][joint] += R_sb[row][k] * b_local[k][joint];
      }
    }
  }

  /* ================================================================
   * 5. Platform position
   * ================================================================ */

  double Z_abs = L_resting + stroke_home + z_rel_clamped;

  /*
   * MATLAB:
   *
   * Px = -(u(1,1) + u(1,2) + u(1,3)) / 3
   * Py = -(u(2,1) + u(2,2) + u(2,3)) / 3
   */
  double Px = -(u[0][0] + u[0][1] + u[0][2]) / 3.0;

  double Py = -(u[1][0] + u[1][1] + u[1][2]) / 3.0;

  double P[3] = {Px, Py, Z_abs};

  /* ================================================================
   * 6. Calculate actuator lengths
   * ================================================================ */

  for (int joint = 0; joint < 3; joint++) {
    /*
     * MATLAB:
     *
     * b_air = P + u;
     *
     * d_leg = b_air - a;
     */

    double b_air_x = P[0] + u[0][joint];
    double b_air_y = P[1] + u[1][joint];
    double b_air_z = P[2] + u[2][joint];

    double d_leg_x = b_air_x - a[0][joint];

    double d_leg_y = b_air_y - a[1][joint];

    double d_leg_z = b_air_z - a[2][joint];

    /*
     * MATLAB:
     *
     * L = sqrt(sum(d_leg.^2, 1))
     */
    double L =
        sqrt((d_leg_x * d_leg_x) + (d_leg_y * d_leg_y) + (d_leg_z * d_leg_z));

    /*
     * Convert total leg length to actuator stroke.
     */
    q[joint] = L - L_resting;

    /*
     * Safe stroke clamp:
     *
     * MATLAB:
     * q = max(min(q, 150), 0);
     */
    if (q[joint] > 150.0) {
      q[joint] = 150.0;
    } else if (q[joint] < 0.0) {
      q[joint] = 0.0;
    }
  }
}

void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
  while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) < us); // wait for the counter to reach the us input in the parameter
}


void Stepper_Move3(StepperMotor *motor1, GPIO_PinState dir1, uint32_t steps1,
                   StepperMotor *motor2, GPIO_PinState dir2, uint32_t steps2,
                   StepperMotor *motor3, GPIO_PinState dir3, uint32_t steps3) 
                   {
  /* ---------------------------------
     Set directions first
     --------------------------------- */

  HAL_GPIO_WritePin(motor1->DIR_Port, motor1->DIR_Pin, dir1);

  HAL_GPIO_WritePin(motor2->DIR_Port, motor2->DIR_Pin, dir2);

  HAL_GPIO_WritePin(motor3->DIR_Port, motor3->DIR_Pin, dir3);

  //HAL_Delay(1);
  delay_us(500); 

  /* ---------------------------------
     Find largest requested step count
     --------------------------------- */

  uint32_t maxSteps = steps1;

  if (steps2 > maxSteps)
    maxSteps = steps2;

  if (steps3 > maxSteps)
    maxSteps = steps3;

  /* ---------------------------------
     Generate steps simultaneously
     --------------------------------- */

  for (uint32_t i = 0; i < maxSteps; i++) {
    /*
     * Raise STEP only for motors
     * which still need to move.
     */

    if (i < steps1) {
      HAL_GPIO_WritePin(motor1->STEP_Port, motor1->STEP_Pin, GPIO_PIN_SET);
    }

    if (i < steps2) {
      HAL_GPIO_WritePin(motor2->STEP_Port, motor2->STEP_Pin, GPIO_PIN_SET);
    }

    if (i < steps3) {
      HAL_GPIO_WritePin(motor3->STEP_Port, motor3->STEP_Pin, GPIO_PIN_SET);
    }

    //HAL_Delay(1);
    delay_us(500); 
    

        /*
         * Bring STEP signals LOW
         */

        if (i < steps1) {
      HAL_GPIO_WritePin(motor1->STEP_Port, motor1->STEP_Pin, GPIO_PIN_RESET);
    }

    if (i < steps2) {
      HAL_GPIO_WritePin(motor2->STEP_Port, motor2->STEP_Pin, GPIO_PIN_RESET);
    }

    if (i < steps3) {
      HAL_GPIO_WritePin(motor3->STEP_Port, motor3->STEP_Pin, GPIO_PIN_RESET);
    }

    delay_us(500);

  }
}

void MoveActuatorsToTarget(double q[3]) {
  /*
   * q[] contains ABSOLUTE TARGET STROKE positions:
   *
   * q[0] -> target stroke of actuator 1 [mm]
   * q[1] -> target stroke of actuator 2 [mm]
   * q[2] -> target stroke of actuator 3 [mm]
   */

  double move1 = q[0] - FrontActuator.current_stroke_length_mm;

  double move2 = q[1] - BackRightActuator.current_stroke_length_mm;

  double move3 = q[2] - BackLeftActuator.current_stroke_length_mm;

  /*
   * Positive movement = extend
   * Negative movement = retract
   *
   * For now assuming GPIO_PIN_SET means EXTEND.
   */
  GPIO_PinState dir1 = (move1 >= 0.0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  GPIO_PinState dir2 = (move2 >= 0.0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  GPIO_PinState dir3 = (move3 >= 0.0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  /*
   * Convert required movement [mm] to step pulses.
   */
  uint32_t steps1 = (uint32_t)(fabs(move1) * STEPS_PER_MM + 0.5);

  uint32_t steps2 = (uint32_t)(fabs(move2) * STEPS_PER_MM + 0.5);

  uint32_t steps3 = (uint32_t)(fabs(move3) * STEPS_PER_MM + 0.5);

  /*
   * Physically move all three actuators.
   */
  Stepper_Move3(&FrontActuator, dir1, steps1, &BackRightActuator, dir2, steps2, &BackLeftActuator,
                dir3, steps3);

  /*
   * Update software-estimated positions.
   *
   * We use the number of steps actually commanded rather
   * than simply setting current position equal to q.
   */

  double moved1 = (double)steps1 / STEPS_PER_MM;

  double moved2 = (double)steps2 / STEPS_PER_MM;

  double moved3 = (double)steps3 / STEPS_PER_MM;

  if (dir1 == GPIO_PIN_SET)
    FrontActuator.current_stroke_length_mm += moved1;
  else
    FrontActuator.current_stroke_length_mm -= moved1;

  if (dir2 == GPIO_PIN_SET)
    BackRightActuator.current_stroke_length_mm += moved2;
  else
    BackRightActuator.current_stroke_length_mm -= moved2;

  if (dir3 == GPIO_PIN_SET)
    BackLeftActuator.current_stroke_length_mm += moved3;
  else
    BackLeftActuator.current_stroke_length_mm -= moved3;
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start(&htim1); // start the Timer1

  BNO055_Attach(&bno, &hi2c1, &hcom_uart[COM1],
                BNO055_I2C_ADDRESS_7BIT_DEFAULT);

  BNO055_Status_t status;

  status = BNO055_Begin(&bno);

  if (status != BNO055_STATUS_OK) {
    BNO055_PrintStatus(&bno, status);
  }

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

  double Z_cmd, roll_cmd, pitch_cmd;
  while (1)
  {

    //TEST STEPPER MOVE - UNCOMMENT TO TEST
    // Stepper_Move3(&Actuator1, GPIO_PIN_SET, 200, &Actuator2, GPIO_PIN_SET, 200,
    //               &Actuator3, GPIO_PIN_SET, 200);
    // HAL_Delay(2);
    // Stepper_Move3(&Actuator1, GPIO_PIN_RESET, 200, &Actuator2, GPIO_PIN_RESET, 200,
    //               &Actuator3, GPIO_PIN_RESET, 200);
    // HAL_Delay(2);

    //IMU CONTROL - UNCOMMENT TO USE IMU FOR CONTROL
    if (BNO055_ReadEuler(&bno, &bno_euler) == BNO055_STATUS_OK) {
      BNO055_PrintEuler(&bno, &bno_euler);
      roll = bno_euler.roll;
      pitch = bno_euler.pitch;
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

    //BUTTON testing
    GPIO_PinState buttonState =
    HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);

    if (buttonState == GPIO_PIN_SET) {
      /* Button pressed */
      BSP_LED_On(LED_GREEN);
    } else {
      /* Button released */
      BSP_LED_Off(LED_GREEN);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 240-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STEP_OUT_ACT3_Pin|DIR_OUT_ACT2_Pin|DIR_OUT_ACT1_Pin|STEP_OUT_ACT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DIR_OUT_ACT3_Pin|STEP_OUT_ACT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : START_BUTTON_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_OUT_ACT3_Pin DIR_OUT_ACT2_Pin DIR_OUT_ACT1_Pin STEP_OUT_ACT2_Pin */
  GPIO_InitStruct.Pin = STEP_OUT_ACT3_Pin|DIR_OUT_ACT2_Pin|DIR_OUT_ACT1_Pin|STEP_OUT_ACT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : STOP_BUTTON_Pin */
  GPIO_InitStruct.Pin = STOP_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STOP_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_OUT_ACT3_Pin STEP_OUT_ACT1_Pin */
  GPIO_InitStruct.Pin = DIR_OUT_ACT3_Pin|STEP_OUT_ACT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
