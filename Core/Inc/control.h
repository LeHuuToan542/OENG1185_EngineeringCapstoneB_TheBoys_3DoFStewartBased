/*
 * control.h
 *
 * Control Module: decides the target platform pose from the active input
 * source (IMU / PuTTY), runs IK, and drives the actuators.
 *
 * Wrapped in a two-state machine (IDLE / RUNNING) driven by the START and
 * STOP push buttons. Nothing reaches the motor drivers while IDLE.
 */

#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Target/measured platform pose. */
extern double Z;
extern double roll;
extern double pitch;

/* Latest computed actuator stroke targets [mm], from simscape_ik(). */
extern double q[3];

/* Enter IDLE and announce it. Call once before the main loop. */
void Control_Init(void);

/*
 * Button events. Safe to call from an EXTI callback: they only raise a
 * flag, which Control_Update() acts on in the main loop.
 */
void Control_StartRequest(void);
void Control_StopRequest(void);

/* Call once per main loop iteration. */
void Control_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
