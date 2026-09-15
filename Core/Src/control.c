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

double q[3];

double Z = 50;
double roll = 0;
double pitch = 0;

/*
 * Mutually exclusive control source, switched over PuTTY: typing "IMU"
 * enables IMU mirror mode, and any "Z,roll,pitch" command switches back
 * to PuTTY mode. Starts in PuTTY mode so nothing moves until commanded.
 */
static int imu_mode = 0;

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

static void EnterIdle(void) {
  state = STATE_IDLE;
  imu_mode = 0;

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

  /* Release the emergency stop so moves are allowed again. */
  Drive_ClearAbort();

  BSP_LED_Off(LED_YELLOW);
  BSP_LED_On(LED_GREEN);

  Comms_Print("\r\n=== RUNNING ===\r\n");
  Comms_SendWelcomeMessage();
}

void Control_Init(void) {
  /*
   * Home once at startup, before the system is ready to accept commands.
   * Deliberately NOT re-run on every later EnterIdle() (e.g. a STOP press):
   * an emergency stop must only halt motion, never kick off a new move.
   */
  Comms_Print("\r\n=== HOMING === Retracting all actuators to lower limit...\r\n");

  if (Drive_HomeAll()) {
    Comms_Print("=== HOMING COMPLETE ===\r\n");
  } else {
    Comms_Print("=== HOMING FAILED / ABORTED === Check limit switches and wiring.\r\n");
  }

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

  int cmd = Serial_ReadPoseCommand(&Z_cmd, &roll_cmd, &pitch_cmd);

  if (cmd == 2) {
    imu_mode = 1;
    Comms_Print("\r\nIMU MODE ENABLED\r\n> ");
  } else if (cmd == 1) {
    imu_mode = 0;
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
  } else if (cmd == -1) {
    Comms_Print("\r\nINVALID COMMAND. Expected Z,roll,pitch (e.g. 10,5,-3) or IMU\r\n> ");
  }

  if (imu_mode) {
    if (Comms_ReadIMU(&roll, &pitch)) {
      simscape_ik(Z, roll, pitch, q);
      MoveActuatorsToTarget(q);
    }
  }
}
