/*
 * ik.c
 *
 * IK Module: pure inverse-kinematics math for the 3-DoF Stewart platform.
 * Relocated from main.c without behavior changes.
 */

#include "ik.h"
#include <math.h>

void simscape_ik(double Z, double roll, double pitch, double q[3]) {
  /* ================================================================
   * 1. Geometry
   * ================================================================ */

  const double r = IK_PLATFORM_RADIUS_MM; // Platform/base radius [mm]

  const double L_resting = IK_L_RESTING_MM;
  const double stroke_home = IK_STROKE_HOME_MM;

  /* ================================================================
   * 2. Input clamping
   * ================================================================ */

  const double max_tilt_deg = IK_MAX_TILT_DEG;

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
   * Limit Z to 40 mm.
   */
  double z_rel_clamped = Z;

  if (z_rel_clamped > IK_Z_CLAMP_MM) {
    z_rel_clamped = IK_Z_CLAMP_MM;
  }

  /*
   * Convert degrees to radians.
   */
  double rad_roll = roll * (IK_PI / 180.0);
  double rad_pitch = pitch * (IK_PI / 180.0);

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
    double angle_rad = angles_deg[i] * (IK_PI / 180.0);

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
    if (q[joint] > IK_STROKE_MAX_MM) {
      q[joint] = IK_STROKE_MAX_MM;
    } else if (q[joint] < IK_STROKE_MIN_MM) {
      q[joint] = IK_STROKE_MIN_MM;
    }
  }
}
