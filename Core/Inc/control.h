/*
 * control.h
 *
 * Control Module: decides the target platform pose from the active input
 * source (IMU / PuTTY), runs IK, and drives the actuators.
 *
 * Currently just relocates the existing IMU/PuTTY decision logic from
 * main.c's loop as-is. This is the module to grow into a real state
 * machine later.
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

/* Call once per main loop iteration. */
void Control_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
