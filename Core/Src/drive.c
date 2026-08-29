/*
 * drive.c
 *
 * Drive Module: dedicated stepper driver control. Relocated from main.c
 * without behavior changes.
 */

#include "drive.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

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
