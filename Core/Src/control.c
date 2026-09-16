/*
 * control.c
 *
 * Control Module. Runs the IMU/PuTTY pose logic inside a two-state machine
 * (IDLE / RUNNING) driven by the START and STOP push buttons.
 */

#include "control.h"
#include "comms.h"
#include "ik.h"
#include "drive.h"
#include <stdio.h>

double q[3];

double Z = 0;
double roll = 0;
double pitch = 0;

/*
 * Mutually exclusive control source, switched over PuTTY: typing "IMU"
 * enables IMU mirror mode, and any "Z,roll,pitch" command switches back
 * to PuTTY mode. Starts in PuTTY mode so nothing moves until commanded.
 */
static int imu_mode = 0;

/*
 * Limit switch monitor, entered by typing "LIM". Reports the six switches
 * whenever one changes so they can be pressed by hand and checked. Any other
 * command leaves it again.
 */
static int limit_mode = 0;
static uint8_t limit_bits = 0;

typedef enum {
  STATE_IDLE = 0,
  STATE_RUNNING
} SysState_t;

static SysState_t state = STATE_IDLE;

/*
 * Set by the button EXTI callback, cleared by Control_Update(). Keeps the
 * ISR free of UART transmits and motor calls.
 */
static volatile uint8_t start_req = 0;
static volatile uint8_t stop_req = 0;

/* Mechanical buttons bounce for a few ms; ignore repeats within this window. */
#define BUTTON_DEBOUNCE_MS 200u

static uint8_t Limits_Snapshot(void) {
  uint8_t bits = 0;

  for (int m = 0; m < ACTUATOR_COUNT; m++) {
    if (Drive_UpperLimitHit(m)) {
      bits |= (uint8_t)(1u << (2 * m));
    }

    if (Drive_LowerLimitHit(m)) {
      bits |= (uint8_t)(1u << (2 * m + 1));
    }
  }

  return bits;
}

static void Limits_Report(void) {
  char msg[96];

  int len = snprintf(
      msg, sizeof(msg),
      "LIMITS  A1 up=%d low=%d  A2 up=%d low=%d  A3 up=%d low=%d\r\n",
      Drive_UpperLimitHit(0), Drive_LowerLimitHit(0), Drive_UpperLimitHit(1),
      Drive_LowerLimitHit(1), Drive_UpperLimitHit(2), Drive_LowerLimitHit(2));

  if (len > 0) {
    Comms_Print(msg);
  }
}

static void EnterIdle(void) {
  state = STATE_IDLE;
  imu_mode = 0;
  limit_mode = 0;

  /*
   * Latch the stop, so any move that somehow starts while IDLE is refused
   * outright. Released again by EnterRunning().
   */
  Drive_AbortRequest();

  BSP_LED_Off(LED_GREEN);
  BSP_LED_On(LED_YELLOW);

  Comms_Print("\r\n=== STOPPED / IDLE === Motors halted. Press START to run.\r\n");
}

static void EnterRunning(void) {
  state = STATE_RUNNING;
  imu_mode = 0;
  limit_mode = 0;

  /* Release the emergency stop so moves are allowed again. */
  Drive_ClearAbort();

  BSP_LED_Off(LED_YELLOW);
  BSP_LED_On(LED_GREEN);

  Comms_Print("\r\n=== RUNNING ===\r\n");
  Comms_SendWelcomeMessage();
}

void Control_Init(void) {
  EnterIdle();
}

void Control_StartRequest(void) {
  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();

  if ((now - last_tick) < BUTTON_DEBOUNCE_MS) {
    return;
  }

  last_tick = now;
  start_req = 1;
}

void Control_StopRequest(void) {
  /*
   * Kill the pulse train first, before any debounce test. A move in
   * progress blocks the main loop, so waiting for Control_Update() to see
   * stop_req would let the platform finish its travel. Repeated calls from
   * a bouncing contact are harmless here - the flag just gets re-set.
   */
  Drive_AbortRequest();

  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();

  if ((now - last_tick) < BUTTON_DEBOUNCE_MS) {
    return;
  }

  last_tick = now;
  stop_req = 1;
}

void Control_Update(void) {
  double Z_cmd, roll_cmd, pitch_cmd;
  int test_actuator, test_dir;
  double test_mm;

  /*
   * STOP wins over START so a simultaneous press leaves the platform safe.
   */
  if (stop_req) {
    stop_req = 0;
    start_req = 0;

    if (state != STATE_IDLE) {
      EnterIdle();
    }
  } else if (start_req) {
    start_req = 0;

    if (state != STATE_RUNNING) {
      EnterRunning();
    }
  }

  if (state != STATE_RUNNING) {
    return;
  }

  int cmd = Serial_ReadPoseCommand(&Z_cmd, &roll_cmd, &pitch_cmd,
                                   &test_actuator, &test_dir, &test_mm);

  if (cmd == 2) {
    imu_mode = 1;
    limit_mode = 0;
    Comms_Print("\r\nIMU MODE ENABLED\r\n> ");
  } else if (cmd == 4) {
    imu_mode = 0;
    limit_mode = 1;
    limit_bits = Limits_Snapshot();

    Comms_Print("\r\nLIMIT SWITCH MONITOR - press each switch by hand.\r\n"
                "Any other command exits.\r\n");
    Limits_Report();
  } else if (cmd == 1) {
    imu_mode = 0;
    limit_mode = 0;
    /*
     * Save new desired platform pose.
     */
    Z = Z_cmd;
    roll = roll_cmd;
    pitch = pitch_cmd;
    Comms_Print("POSE COMMAND RECEIVED\r\n");
    simscape_ik(Z, roll, pitch, q);
    MoveActuatorsToTarget(q);
    UART_SendLegLengths(q);
    Comms_Print("\r\nEnter next command:\r\n> ");
  } else if (cmd == 3) {
    imu_mode = 0;
    limit_mode = 0;

    /*
     * Drive only the selected actuator: zero steps for the others means
     * Stepper_Move3() never pulses their STEP lines.
     */
    GPIO_PinState dir[ACTUATOR_COUNT] = {GPIO_PIN_RESET, GPIO_PIN_RESET,
                                         GPIO_PIN_RESET};
    uint32_t steps[ACTUATOR_COUNT] = {0, 0, 0};

    dir[test_actuator] = test_dir ? GPIO_PIN_SET : GPIO_PIN_RESET;
    steps[test_actuator] = (uint32_t)(test_mm * STEPS_PER_MM + 0.5);

    char msg[64];
    int len = snprintf(msg, sizeof(msg), "TESTING ACTUATOR %d: %.2f mm %s\r\n",
                       test_actuator + 1, test_mm,
                       test_dir ? "EXTEND" : "RETRACT");
    if (len > 0) {
      Comms_Print(msg);
    }

    uint32_t done = Stepper_Move3(Actuators, dir, steps);

    double moved = (double)done / STEPS_PER_MM;

    if (test_dir)
      Actuators[test_actuator]->current_stroke_length_mm += moved;
    else
      Actuators[test_actuator]->current_stroke_length_mm -= moved;

    Comms_Print("\r\nEnter next command:\r\n> ");
  } else if (cmd == -1) {
    Comms_Print("\r\nINVALID COMMAND. Expected Z,roll,pitch (e.g. 10,5,-3), "
               "An,mm,dir (e.g. A1,10,1), LIM, or IMU\r\n> ");
  }

  if (limit_mode) {
    uint8_t bits = Limits_Snapshot();

    if (bits != limit_bits) {
      limit_bits = bits;
      Limits_Report();
    }
  }

  if (imu_mode) {
    if (Comms_ReadIMU(&roll, &pitch)) {
      simscape_ik(Z, roll, pitch, q);
      MoveActuatorsToTarget(q);
    }
  }
}
