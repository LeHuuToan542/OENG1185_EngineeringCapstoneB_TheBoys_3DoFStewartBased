/*
 * ik.h
 *
 * IK Module: pure inverse-kinematics math for the 3-DoF Stewart platform,
 * plus the hardware geometry configuration it depends on.
 *
 * Tune the platform/actuator geometry here.
 */

#ifndef IK_H
#define IK_H

#ifdef __cplusplus
extern "C" {
#endif

#define IK_PI 3.14159265358979323846

/* ---- Hardware configuration (tune for your platform) ---- */

#define IK_PLATFORM_RADIUS_MM   100.0  /* Platform/base radius */
#define IK_STROKE_LENGTH_MM     200.0  /* Actuator stroke specification */
#define IK_L_RESTING_MM         (125.0 + IK_STROKE_LENGTH_MM) /* Leg length at stroke = 0 */
#define IK_STROKE_HOME_MM       0.0

/* ---- Input/output limits ---- */

#define IK_MAX_TILT_DEG         25.0   /* Max combined roll/pitch magnitude */
#define IK_Z_CLAMP_MM           40.0   /* Z is clamped to +/- this value */
#define IK_STROKE_MIN_MM        0.0
#define IK_STROKE_MAX_MM        150.0

/*
 * Compute the three actuator stroke targets q[0..2] [mm] for a requested
 * platform heave Z [mm], roll [deg], and pitch [deg].
 */
void simscape_ik(double Z, double roll, double pitch, double q[3]);

#ifdef __cplusplus
}
#endif

#endif /* IK_H */
