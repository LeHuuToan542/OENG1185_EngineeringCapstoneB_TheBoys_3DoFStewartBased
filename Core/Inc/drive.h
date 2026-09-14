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

/* STEP pulse half-period [us]: one step takes 2 x this. */
#define STEP_PULSE_US 500

/* Number of actuators. */
#define ACTUATOR_COUNT 3

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

/* The three actuators in q[] order: front, back-right, back-left. */
extern StepperMotor *Actuators[ACTUATOR_COUNT];

void delay_us(uint16_t us);

/*
 * Emergency stop. Safe to call from an interrupt: it aborts the pulse loop
 * inside Stepper_Move3() within one step period instead of waiting for the
 * whole move to finish. Stays latched until Drive_ClearAbort().
 */
void Drive_AbortRequest(void);

/* Clear a latched abort so moves are allowed again. */
void Drive_ClearAbort(void);

/* Non-zero while an abort is latched. */
int Drive_Aborted(void);

/*
 * Step all three actuators together: motors[i] moves steps[i] pulses in
 * direction dir[i]. Returns once the longest of the three has finished, or
 * early if Drive_AbortRequest() fires. Returns the number of step pulses
 * actually issued, so the caller can keep its position estimate honest.
 */
uint32_t Stepper_Move3(StepperMotor *motors[ACTUATOR_COUNT],
                       const GPIO_PinState dir[ACTUATOR_COUNT],
                       const uint32_t steps[ACTUATOR_COUNT]);

/*
 * Move all three actuators to the absolute target strokes in q[0..2] [mm].
 */
void MoveActuatorsToTarget(double q[3]);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
