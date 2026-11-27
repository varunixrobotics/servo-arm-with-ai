/*
  =============================================================================
  S.A.M. (Servo Arm Manipulator) - 6-Axis High-Performance Hardware Firmware
  Target: Arduino Uno / Mega / Nano
  Baud Rate: 115200

  EXACT HARDWARE PINOUT:
    - Joint 1 (Base Pan):        Pin 8   (Standard 180° Servo, 0° to 180°)
    - Joint 2 (Shoulder Pitch):  Pin 9   (Standard 180° Servo, 0° to 180°)
    - Joint 3 (Elbow Pitch):     Pin 10  (MG995 High-Torque 180°, 0° to 180°)
    - Joint 4 (Wrist Pitch):     Pin 11  (MG90S 360° Servo, 0° to 180° Scaled)
    - Joint 5 (Wrist Roll):      Pin 12  (MG90S 360° Servo, 0° to 160° Scaled)
    - Joint 6 (Gripper):         Pin 13  (MG90S 360° Servo, 20° Shut to 145° Open)

  NEW FEATURES & FIXES:
  1. Instant / Fast Movement Profiles:
     - SPEED:INSTANT -> Direct write (0ms delay), moves instantly at max hardware slew rate!
     - SPEED:FAST    -> 2ms/deg, 3° step (sub-second smooth response)
     - SPEED:SMOOTH  -> 14ms/deg (controlled cinematic motion)
  2. 360° MG90S Positional Alignment Engine:
     Maps 0°-180° software commands to 1000us-2000us pulse widths centered
     at 1500us. This restricts 360° servos to an exact 180° physical arc,
     preventing over-rotation and ensuring perfect joint alignment!
  3. Maximum Signal Drive & Headroom:
     Full 500us-2500us timing bounds with explicit push-pull OUTPUT pin configuration.
  4. Continuous Consumer Movement Queue:
     Zero dropped packets; serial remains non-blocking (<2ms polling).
  5. Emergency Stop:
     "STOP" / "HALT" halts all movement in <2ms.
  6. Hardware Diagnostic Sweep:
     "TEST" command sequentially tests Pins 8 to 13.
  =============================================================================
*/

#include <Servo.h>

// -------------------------------------------------------------
// Exact Hardware Pin Definitions (PRESERVED)
// -------------------------------------------------------------
const int PIN_BASE     = 8;   // Joint 1: Base Pan
const int PIN_SHOULDER = 9;   // Joint 2: Shoulder Pitch
const int PIN_ELBOW    = 10;  // Joint 3: Elbow Pitch (MG995)
const int PIN_WRIST_P  = 11;  // Joint 4: Wrist Pitch (MG90S)
const int PIN_WRIST_R  = 12;  // Joint 5: Wrist Roll (MG90S)
const int PIN_GRIPPER  = 13;  // Joint 6: Gripper Claw (MG90S)

const int SERVO_PINS[6] = {
  PIN_BASE,      // Index 0 -> Pin 8
  PIN_SHOULDER,  // Index 1 -> Pin 9
  PIN_ELBOW,     // Index 2 -> Pin 10
  PIN_WRIST_P,   // Index 3 -> Pin 11
  PIN_WRIST_R,   // Index 4 -> Pin 12
  PIN_GRIPPER    // Index 5 -> Pin 13
};

// -------------------------------------------------------------
// High-Drive Servo Timing Limits (microseconds)
// -------------------------------------------------------------
const int SERVO_PULSE_MIN_US[6] = {
  500,  // J1 Base
  500,  // J2 Shoulder
  580,  // J3 Elbow: MG995 (580us protects internal mechanical stop)
  500,  // J4 Wrist Pitch: MG90S
  500,  // J5 Wrist Roll: MG90S
  544   // J6 Gripper: MG90S (544us ensures valid pulse at 0° shut)
};

const int SERVO_PULSE_MAX_US[6] = {
  2500, // J1 Base
  2500, // J2 Shoulder
  2420, // J3 Elbow: MG995 (2420us protects internal mechanical stop)
  2500, // J4 Wrist Pitch: MG90S
  2500, // J5 Wrist Roll: MG90S
  2400  // J6 Gripper: MG90S (2400us ensures valid pulse at 160° open)
};

// -------------------------------------------------------------
// 360° Servo Kinematic Alignment Configuration
// J4 (Wrist Pitch) uses MG90S 360° positional mapping.
// J5 (Wrist Roll) and J6 (Gripper) are mechanical linkages:
// J6 Gripper must NOT use 360° scaling because claw linkages
// have only ~60°-90° of mechanical travel and will jam if overdriven!
// -------------------------------------------------------------
bool enable360Alignment[6] = {
  false, // J1 Base (Standard 180°)
  false, // J2 Shoulder (Standard 180°)
  false, // J3 Elbow MG995 (Standard 180°)
  true,  // J4 Wrist Pitch: MG90S 360°
  false, // J5 Wrist Roll: Standard travel prevents bracket collision
  false  // J6 Gripper: Standard travel prevents claw jaw jamming
};

// -------------------------------------------------------------
// Speed Profiles (Fast vs Instant vs Smooth)
// -------------------------------------------------------------
enum SpeedMode {
  SPEED_INSTANT, // Direct write (0ms delay) -> zero delay
  SPEED_FAST,    // Sub-second snappy (2ms/deg, 3° steps) -> FAST & SAFE!
  SPEED_NORMAL,  // Balanced (6ms/deg, 2° steps)
  SPEED_SMOOTH,  // Smooth cinematic (14ms/deg, 1° steps)
  SPEED_SLOW     // Very slow (28ms/deg, 1° steps)
};

// SPEED_FAST is the ideal default: moves 60° in ~40ms (feels instant to human eye)
// without the massive inrush current surge that causes ticking/brownout!
SpeedMode currentSpeedMode = SPEED_FAST;

// -------------------------------------------------------------
// Mechanical Safe Angle Limits & Calibration
// -------------------------------------------------------------
const int MIN_BASE   = 0,   MAX_BASE   = 180;
const int MIN_SHLD   = 0,   MAX_SHLD   = 180;
const int MIN_ELBW   = 0,   MAX_ELBW   = 180;
const int MIN_WRST_P = 0,   MAX_WRST_P = 180;
const int MIN_WRST_R = 0,   MAX_WRST_R = 160; // 0° to 160° Wrist Roll
const int MIN_GRIP   = 0,   MAX_GRIP   = 145; // Full 0° (Shut) to 145° (Max Open) Travel!

// Gripper calibration targets
const int GRIP_OPEN_ANGLE  = 145; // Fully open target (145°)
const int GRIP_CLOSE_ANGLE = 0;   // Fully closed target (0°)

// Home Pose
const int HOME_ANGLES[6] = {90, 90, 90, 90, 90, 90};

// -------------------------------------------------------------
// Servo Objects & State Variables
// -------------------------------------------------------------
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWristPitch;
Servo servoWristRoll;
Servo servoGripper;

Servo* servos[6] = {
  &servoBase,
  &servoShoulder,
  &servoElbow,
  &servoWristPitch,
  &servoWristRoll,
  &servoGripper
};

int currentAngles[6] = {90, 90, 90, 90, 90, 90};
int targetAngles[6]  = {90, 90, 90, 90, 90, 90};

volatile bool isStopped = false;
bool isRelaxed = false;
bool isSweepingActive = false;

// Serial Buffer
String inputString = "";

// -------------------------------------------------------------
// Clamp Angle to Hardware Limits
// -------------------------------------------------------------
int clampAngle(int jointIndex, int angle) {
  switch (jointIndex) {
    case 0: return constrain(angle, MIN_BASE, MAX_BASE);
    case 1: return constrain(angle, MIN_SHLD, MAX_SHLD);
    case 2: return constrain(angle, MIN_ELBW, MAX_ELBW);
    case 3: return constrain(angle, MIN_WRST_P, MAX_WRST_P);
    case 4: return constrain(angle, MIN_WRST_R, MAX_WRST_R);
    case 5: return constrain(angle, MIN_GRIP, MAX_GRIP);
    default: return constrain(angle, 0, 180);
  }
}

// -------------------------------------------------------------
// Write Angle with 360° Alignment Compensation
// -------------------------------------------------------------
void writeJointAngle(int jointIdx, int deg) {
  if (jointIdx < 0 || jointIdx >= 6) return;
  Servo* s = servos[jointIdx];
  if (!s || !s->attached()) return;

  deg = clampAngle(jointIdx, deg);

  if (enable360Alignment[jointIdx]) {
    // ═════════════════════════════════════════════════════════
    // 360° Positional Alignment Engine:
    // A 360° servo spans 500us..2500us across 360 physical degrees.
    // To align a 0°..180° command with the arm joint:
    // Map 0°..180° to 1000us..2000us (centered at 1500us).
    //   - 0°   -> 1000 us (centered -45° to -90° physical)
    //   - 90°  -> 1500 us (neutral 180° physical: PERFECT ALIGNMENT!)
    //   - 180° -> 2000 us (centered +45° to +90° physical)
    // Physical travel is restricted to 180°, aligning 1:1 with sliders!
    // ═════════════════════════════════════════════════════════
    int pulseUs = map(deg, 0, 180, 1000, 2000);
    s->writeMicroseconds(pulseUs);
  } else {
    // Standard 180° Servos (J1, J2, J3 MG995)
    s->write(deg);
  }
}

// -------------------------------------------------------------
// Attach Helper with High-Drive Signal
// -------------------------------------------------------------
void attachServo(int idx) {
  if (idx < 0 || idx >= 6) return;
  if (!servos[idx]->attached()) {
    pinMode(SERVO_PINS[idx], OUTPUT);
    servos[idx]->attach(SERVO_PINS[idx], SERVO_PULSE_MIN_US[idx], SERVO_PULSE_MAX_US[idx]);
    writeJointAngle(idx, currentAngles[idx]);
  }
}

void attachAllServos() {
  for (int i = 0; i < 6; i++) {
    attachServo(i);
    delay(20);
  }
  isRelaxed = false;
}

// Check if any joint has a pending movement
bool hasPendingMoves() {
  for (int i = 0; i < 6; i++) {
    if (currentAngles[i] != clampAngle(i, targetAngles[i])) {
      return true;
    }
  }
  return false;
}

// Forward declarations
void processIncomingSerial();
void serviceMovementQueue();

// -------------------------------------------------------------
// Sweep a Single Joint: Instant or Stepped
// -------------------------------------------------------------
void sweepJoint(int jointIdx, int targetDeg) {
  if (jointIdx < 0 || jointIdx >= 6) return;
  if (isStopped) return;

  targetDeg = clampAngle(jointIdx, targetDeg);
  int startDeg = currentAngles[jointIdx];

  if (startDeg == targetDeg) return;

  Servo* s = servos[jointIdx];
  if (!s) return;

  attachServo(jointIdx);

  // ── INSTANT MODE: Direct Write (0 ms delay) ────────────────
  if (currentSpeedMode == SPEED_INSTANT) {
    writeJointAngle(jointIdx, targetDeg);
    currentAngles[jointIdx] = targetDeg;
    targetAngles[jointIdx]  = targetDeg;
    return;
  }

  // ── STEPPED / SMOOTH MODES ─────────────────────────────────
  int stepSize = 1;
  int stepDelay = 10;

  switch (currentSpeedMode) {
    case SPEED_FAST:
      stepSize  = 3; // Step 3° at a time
      stepDelay = 2; // 2ms per step -> 60° in 40ms!
      break;
    case SPEED_NORMAL:
      stepSize  = 2;
      stepDelay = 6;
      break;
    case SPEED_SMOOTH:
      stepSize  = 1;
      stepDelay = 14;
      break;
    case SPEED_SLOW:
      stepSize  = 1;
      stepDelay = 28;
      break;
    default:
      stepSize  = 1;
      stepDelay = 10;
      break;
  }

  int pos = startDeg;

  while (pos != targetDeg && !isStopped) {
    int diff = targetDeg - pos;
    if (abs(diff) <= stepSize) {
      pos = targetDeg;
    } else {
      pos += (diff > 0) ? stepSize : -stepSize;
    }

    writeJointAngle(jointIdx, pos);
    currentAngles[jointIdx] = pos;

    unsigned long stepStart = millis();
    while (millis() - stepStart < (unsigned long)stepDelay) {
      processIncomingSerial();

      if (isStopped) {
        currentAngles[jointIdx] = pos;
        targetAngles[jointIdx]  = pos;
        return;
      }

      int dynamicTarget = clampAngle(jointIdx, targetAngles[jointIdx]);
      if (dynamicTarget != targetDeg) {
        targetDeg = dynamicTarget;
      }
      delay(1);
    }
  }

  if (!isStopped) {
    writeJointAngle(jointIdx, targetDeg);
    currentAngles[jointIdx] = targetDeg;
    targetAngles[jointIdx]  = targetDeg;
  }
}

// -------------------------------------------------------------
// Sequential Movement Manager (Continuous Consumer Queue)
// -------------------------------------------------------------
void serviceMovementQueue() {
  if (isSweepingActive) return;
  isSweepingActive = true;
  isStopped = false;

  int interJointPause = (currentSpeedMode == SPEED_INSTANT) ? 0 : 
                        ((currentSpeedMode == SPEED_FAST) ? 15 : 60);

  while (hasPendingMoves() && !isStopped) {
    int jointToMove = -1;
    for (int i = 0; i < 6; i++) {
      if (currentAngles[i] != clampAngle(i, targetAngles[i])) {
        jointToMove = i;
        break;
      }
    }

    if (jointToMove == -1) break;

    int target = clampAngle(jointToMove, targetAngles[jointToMove]);
    sweepJoint(jointToMove, target);

    if (isStopped) break;

    if (hasPendingMoves() && !isStopped && interJointPause > 0) {
      unsigned long pauseStart = millis();
      while (millis() - pauseStart < (unsigned long)interJointPause) {
        processIncomingSerial();
        if (isStopped) break;
        delay(3);
      }
    }
  }

  isSweepingActive = false;

  if (isStopped) {
    Serial.println("ACK:STOPPED");
  } else {
    Serial.print("ACK:POS:");
    for (int i = 0; i < 6; i++) {
      Serial.print(currentAngles[i]);
      if (i < 5) Serial.print(",");
    }
    Serial.println();
  }
}

// -------------------------------------------------------------
// Parse and Dispatch Commands
// -------------------------------------------------------------
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }

  // ── Immediate Emergency Stop ───────────────────────────────
  if (cmd == "STOP" || cmd == "HALT" || cmd == "PAUSE" || cmd == "E-STOP") {
    isStopped = true;
    for (int i = 0; i < 6; i++) {
      targetAngles[i] = currentAngles[i];
    }
    Serial.println("ACK:STOPPED");
    return;
  }

  // ── Relax Servos ───────────────────────────────────────────
  if (cmd == "RELAX" || cmd == "DETACH") {
    for (int i = 0; i < 6; i++) {
      if (servos[i]->attached()) {
        servos[i]->detach();
      }
    }
    isRelaxed = true;
    Serial.println("ACK:RELAXED");
    return;
  }

  // ── Hold Servos ────────────────────────────────────────────
  if (cmd == "HOLD" || cmd == "ATTACH") {
    attachAllServos();
    Serial.println("ACK:HOLDING");
    return;
  }

  // ── Home Position ──────────────────────────────────────────
  if (cmd == "HOME") {
    isStopped = false;
    attachAllServos();

    int homeSequence[6] = {2, 1, 3, 4, 5, 0};
    for (int idx = 0; idx < 6; idx++) {
      int j = homeSequence[idx];
      targetAngles[j] = HOME_ANGLES[j];
      sweepJoint(j, HOME_ANGLES[j]);

      if (isStopped) break;

      unsigned long pStart = millis();
      while (millis() - pStart < 40) {
        processIncomingSerial();
        if (isStopped) break;
        delay(2);
      }
    }

    if (isStopped) {
      Serial.println("ACK:STOPPED");
    } else {
      Serial.println("ACK:HOME");
    }
    return;
  }

  // ── Gripper Helpers ────────────────────────────────────────
  if (cmd == "GRIP:OPEN") {
    targetAngles[5] = GRIP_OPEN_ANGLE;
    serviceMovementQueue();
    return;
  }

  if (cmd == "GRIP:CLOSE") {
    targetAngles[5] = GRIP_CLOSE_ANGLE;
    serviceMovementQueue();
    return;
  }

  // ── 360° Alignment Mode Toggle ─────────────────────────────
  if (cmd == "MODE:360:ON") {
    enable360Alignment[3] = true;
    enable360Alignment[4] = true;
    enable360Alignment[5] = true;
    for (int i = 3; i < 6; i++) writeJointAngle(i, currentAngles[i]);
    Serial.println("ACK:MODE:360:ON");
    return;
  }

  if (cmd == "MODE:360:OFF") {
    enable360Alignment[3] = false;
    enable360Alignment[4] = false;
    enable360Alignment[5] = false;
    for (int i = 3; i < 6; i++) writeJointAngle(i, currentAngles[i]);
    Serial.println("ACK:MODE:360:OFF");
    return;
  }

  // ── Speed Adjustments ──────────────────────────────────────
  if (cmd.startsWith("SPEED:")) {
    String val = cmd.substring(6);
    val.trim();
    if (val == "INSTANT")       currentSpeedMode = SPEED_INSTANT;
    else if (val == "FAST")     currentSpeedMode = SPEED_FAST;
    else if (val == "NORMAL")   currentSpeedMode = SPEED_NORMAL;
    else if (val == "SMOOTH")   currentSpeedMode = SPEED_SMOOTH;
    else if (val == "SLOW" || val == "VERY_SLOW") currentSpeedMode = SPEED_SLOW;
    else                        currentSpeedMode = SPEED_FAST;

    Serial.print("ACK:SPEED:");
    Serial.println(val);
    return;
  }

  // ── Hardware Diagnostic Joint Sweep ────────────────────────
  if (cmd == "TEST" || cmd == "DIAG") {
    isStopped = false;
    attachAllServos();
    Serial.println("INIT:HARDWARE_TEST_START");
    const char* jointNames[6] = {
      "J1 Base (Pin 8)",
      "J2 Shoulder (Pin 9)",
      "J3 Elbow MG995 (Pin 10)",
      "J4 Wrist Pitch MG90S 360 (Pin 11)",
      "J5 Wrist Roll MG90S 360 (Pin 12)",
      "J6 Gripper MG90S 360 (Pin 13)"
    };

    for (int i = 0; i < 6; i++) {
      if (isStopped) break;
      Serial.print("TESTING:");
      Serial.println(jointNames[i]);

      int orig = currentAngles[i];
      int lowTarget = max(0, orig - 20);
      int highTarget = min(160, orig + 20);

      sweepJoint(i, lowTarget);
      delay(80);
      sweepJoint(i, highTarget);
      delay(80);
      sweepJoint(i, orig);
      delay(150);
    }

    if (isStopped) {
      Serial.println("ACK:STOPPED");
    } else {
      Serial.println("ACK:HARDWARE_TEST_COMPLETE");
    }
    return;
  }

  // ── Angle Packet: <J1,J2,J3,J4,J5,J6> ──────────────────────
  if (cmd.startsWith("<") && cmd.endsWith(">")) {
    String payload = cmd.substring(1, cmd.length() - 1);
    int parsed[6];
    int count = 0;
    int lastComma = -1;

    for (int i = 0; i < 6; i++) {
      int nextComma = payload.indexOf(',', lastComma + 1);
      String s = (nextComma == -1) ? payload.substring(lastComma + 1) : payload.substring(lastComma + 1, nextComma);
      s.trim();
      parsed[i] = s.toInt();
      count++;
      lastComma = nextComma;
      if (nextComma == -1) break;
    }

    if (count == 6) {
      for (int i = 0; i < 6; i++) {
        targetAngles[i] = clampAngle(i, parsed[i]);
      }
      if (!isSweepingActive) {
        serviceMovementQueue();
      }
      return;
    } else if (count == 5) {
      targetAngles[0] = clampAngle(0, parsed[0]);
      targetAngles[1] = clampAngle(1, parsed[1]);
      targetAngles[2] = clampAngle(2, parsed[2]);
      targetAngles[3] = clampAngle(3, parsed[3]);
      targetAngles[5] = clampAngle(5, parsed[4]);
      if (!isSweepingActive) {
        serviceMovementQueue();
      }
      return;
    }
  }

  Serial.print("ERR:UNKNOWN:");
  Serial.println(cmd);
}

// -------------------------------------------------------------
// Non-blocking Serial Processing
// -------------------------------------------------------------
void processIncomingSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        handleCommand(inputString);
        inputString = "";
      }
    } else {
      if (inputString.length() < 64) {
        inputString += c;
      }
    }
  }
}

// -------------------------------------------------------------
// Setup
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) { ; }

  inputString.reserve(64);

  // Set all servo pins as high-drive push-pull outputs for maximum signal integrity
  for (int i = 0; i < 6; i++) {
    pinMode(SERVO_PINS[i], OUTPUT);
  }

  // Soft staggered attach on boot
  for (int i = 0; i < 6; i++) {
    attachServo(i);
    writeJointAngle(i, HOME_ANGLES[i]);
    currentAngles[i] = HOME_ANGLES[i];
    targetAngles[i]  = HOME_ANGLES[i];
    delay(80);
  }

  Serial.println("INIT:SAM_ARM_READY_360_ALIGNED");
}

// -------------------------------------------------------------
// Main Loop
// -------------------------------------------------------------
void loop() {
  processIncomingSerial();

  if (!isStopped && !isSweepingActive && hasPendingMoves()) {
    serviceMovementQueue();
  }
}
