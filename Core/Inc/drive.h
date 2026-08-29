/*
 * drive.h
 *
 * Drive Module: dedicated stepper driver control (step/dir pulse
 * generation, mm <-> steps conversion) for the three actuators.
 */

#ifndef DRIVE_H
#define DRIVE_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Stepper/actuator configuration (tune for your hardware) ---- */

#define MOTOR_STEPS_PER_REV 200.0
#define MICROSTEP 1
#define MM_PER_REV 12.0

#define STEPS_PER_MM ((MOTOR_STEPS_PER_REV * MICROSTEP) / MM_PER_REV)

typedef struct {
  GPIO_TypeDef *STEP_Port;
  uint16_t STEP_Pin;

  GPIO_TypeDef *DIR_Port;
  uint16_t DIR_Pin;

  double current_stroke_length_mm; // Current actuator stroke length in mm.

} StepperMotor;

extern StepperMotor FrontActuator;
extern StepperMotor BackRightActuator;
extern StepperMotor BackLeftActuator;

void delay_us(uint16_t us);

void Stepper_Move3(StepperMotor *motor1, GPIO_PinState dir1, uint32_t steps1,
                   StepperMotor *motor2, GPIO_PinState dir2, uint32_t steps2,
                   StepperMotor *motor3, GPIO_PinState dir3, uint32_t steps3);

/*
 * Move all three actuators to the absolute target strokes in q[0..2] [mm].
 */
void MoveActuatorsToTarget(double q[3]);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
