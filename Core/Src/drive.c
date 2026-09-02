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

StepperMotor *Actuators[ACTUATOR_COUNT] = {&FrontActuator, &BackRightActuator,
                                           &BackLeftActuator};

void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
  while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim1) < us); // wait for the counter to reach the us input in the parameter
}


void Stepper_Move3(StepperMotor *motors[ACTUATOR_COUNT],
                   const GPIO_PinState dir[ACTUATOR_COUNT],
                   const uint32_t steps[ACTUATOR_COUNT]) {
  /* ---------------------------------
     Set directions first
     --------------------------------- */

  uint32_t maxSteps = 0;

  for (int m = 0; m < ACTUATOR_COUNT; m++) {
    HAL_GPIO_WritePin(motors[m]->DIR_Port, motors[m]->DIR_Pin, dir[m]);

    if (steps[m] > maxSteps) {
      maxSteps = steps[m];
    }
  }

  delay_us(STEP_PULSE_US);

  /* ---------------------------------
     Generate steps simultaneously
     --------------------------------- */

  for (uint32_t i = 0; i < maxSteps; i++) {
    /*
     * Raise STEP only for motors which still need to move,
     * then bring the same signals LOW again.
     */

    for (int m = 0; m < ACTUATOR_COUNT; m++) {
      if (i < steps[m]) {
        HAL_GPIO_WritePin(motors[m]->STEP_Port, motors[m]->STEP_Pin,
                          GPIO_PIN_SET);
      }
    }

    delay_us(STEP_PULSE_US);

    for (int m = 0; m < ACTUATOR_COUNT; m++) {
      if (i < steps[m]) {
        HAL_GPIO_WritePin(motors[m]->STEP_Port, motors[m]->STEP_Pin,
                          GPIO_PIN_RESET);
      }
    }

    delay_us(STEP_PULSE_US);
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

  GPIO_PinState dir[ACTUATOR_COUNT];
  uint32_t steps[ACTUATOR_COUNT];

  for (int m = 0; m < ACTUATOR_COUNT; m++) {
    double move = q[m] - Actuators[m]->current_stroke_length_mm;

    /*
     * Positive movement = extend
     * Negative movement = retract
     *
     * For now assuming GPIO_PIN_SET means EXTEND.
     */
    dir[m] = (move >= 0.0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    /*
     * Convert required movement [mm] to step pulses.
     */
    steps[m] = (uint32_t)(fabs(move) * STEPS_PER_MM + 0.5);
  }

  /*
   * Physically move all three actuators.
   */
  Stepper_Move3(Actuators, dir, steps);

  /*
   * Update software-estimated positions.
   *
   * We use the number of steps actually commanded rather
   * than simply setting current position equal to q.
   */

  for (int m = 0; m < ACTUATOR_COUNT; m++) {
    double moved = (double)steps[m] / STEPS_PER_MM;

    if (dir[m] == GPIO_PIN_SET)
      Actuators[m]->current_stroke_length_mm += moved;
    else
      Actuators[m]->current_stroke_length_mm -= moved;
  }
}
