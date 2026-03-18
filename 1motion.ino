#include <Arduino.h>
#include <FlexiTimer2.h>
#include <Servo.h>
#include <math.h> // Ensure math.h is included for nan, sqrt, etc.

// ==========================================
// Leg.h Content
// ==========================================

struct Point {
  float x, y, z;
};

struct LegAngles {
  float coxa, femur, tibia;
};

class Leg {
public:
  // Constructor
  Leg(int coxa_pin, int femur_pin, int tibia_pin, float mounting_x,
      float mounting_y, int leg_index);

  void init();
  void setTarget(Point target_global); // Set target in Global Coordinates
  void update(float dt_s);             // Update motion (interpolation)
  bool atTarget();

  // Getters
  Point getCurrentPos() const { return current_pos; }
  Point getTargetPos() const { return target_pos; }
  Point getMountingPos() const { return {_mounting_x, _mounting_y, 0}; }
  Point getLocalNeutralPos() const { return _neutral_local; }

  // Direct position control (mostly for debugging)
  void moveToLocal(Point local_pos);

private:
  Servo coxa_servo;
  Servo femur_servo;
  Servo tibia_servo;

  int _coxa_pin, _femur_pin, _tibia_pin;
  float _mounting_x, _mounting_y;
  int _leg_index;
  Point _neutral_local; // Stored specific neutral for this leg

  // Current and Target positions in GLOBAL coordinates
  Point current_pos;
  Point target_pos;

  // Internal helper methods
  LegAngles inverseKinematics(Point local_pos);
  LegAngles convertToServoAngles(LegAngles kinematic_angles);

  // Robot Geometry Constants
  static constexpr float COXA_LEN = 27.5;
  static constexpr float FEMUR_LEN = 55.0;
  static constexpr float TIBIA_LEN = 77.5;
  static constexpr float MOVE_SPEED = 150.0; // mm/s - Increased for swing
};

// ==========================================
// GaitEngine.h Content
// ==========================================

class GaitEngine {
public:
  GaitEngine();
  void init();
  void update(); // Called at 50Hz

  // Command Interface
  void commandMoveForward();
  void commandStop();

private:
  Leg *legs[4]; // FR, FL, BR, BL

  // State Machine
  enum GaitState { STOPPED, MOVING_BODY, LIFT_FR, LIFT_BL, LIFT_FL, LIFT_BR };
  GaitState currentState;

  // Gait Parameters
  static constexpr float STRIDE_LENGTH = 40.0; // mm (L)
  static constexpr float LIFT_HEIGHT = 30.0;   // mm

  // Timing
  static constexpr float PHASE_DURATION = 0.5; // seconds per phase (Swing time)

  unsigned long stateStartTime;

  bool runPhase(int swing_leg_index);
};

// ==========================================
// Leg.cpp Content
// ==========================================

Leg::Leg(int coxa_pin, int femur_pin, int tibia_pin, float mounting_x,
         float mounting_y, int leg_index) {
  _coxa_pin = coxa_pin;
  _femur_pin = femur_pin;
  _tibia_pin = tibia_pin;
  _mounting_x = mounting_x;
  _mounting_y = mounting_y;
  _leg_index = leg_index;

  // Define Local Neutral Poses based on Index
  // 0=FR, 1=FL, 2=BR, 3=BL
  // Derived from .bak files: X= +/- 55.0, Y= +/- 52.0, Z= -60.0
  if (_leg_index == 0) { // FR
    _neutral_local = {55.0, 52.0, -60.0};
  } else if (_leg_index == 1) { // FL
    _neutral_local = {-55.0, 52.0, -60.0};
  } else if (_leg_index == 2) { // BR
    _neutral_local = {55.0, -52.0, -60.0};
  } else if (_leg_index == 3) { // BL
    _neutral_local = {-55.0, -52.0, -60.0};
  }

  // Calculate Global Neutral
  current_pos.x = _mounting_x + _neutral_local.x;
  current_pos.y = _mounting_y + _neutral_local.y;
  current_pos.z =
      _mounting_y + _neutral_local.z; // Wait, Z is relative to mount usually.
  // Z logic: "Point localPos = current - mount". So Z is just local Z.
  current_pos.z = -60.0;

  target_pos = current_pos;
}

void Leg::init() {
  coxa_servo.attach(_coxa_pin);
  femur_servo.attach(_femur_pin);
  tibia_servo.attach(_tibia_pin);

  // Move to initial position immediately
  moveToLocal({current_pos.x - _mounting_x, current_pos.y - _mounting_y,
               current_pos.z});
}

void Leg::setTarget(Point target_global) { target_pos = target_global; }

void Leg::update(float dt_s) { // dt_s is time delta in seconds
  // Simple Linear Interpolation

  float diff_x = target_pos.x - current_pos.x;
  float diff_y = target_pos.y - current_pos.y;
  float diff_z = target_pos.z - current_pos.z;

  float dist = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  // If close enough, snap to target
  if (dist < 1.0) {
    current_pos = target_pos;
  } else {
    float move_step = MOVE_SPEED * dt_s;
    if (move_step > dist)
      move_step = dist;

    current_pos.x += (diff_x / dist) * move_step;
    current_pos.y += (diff_y / dist) * move_step;
    current_pos.z += (diff_z / dist) * move_step;
  }

  // Convert Global to Local
  Point local_pos;
  local_pos.x = current_pos.x - _mounting_x;
  local_pos.y = current_pos.y - _mounting_y;
  local_pos.z = current_pos.z; // Z is global relative to body plane (0)

  // Move Servos
  moveToLocal(local_pos);
}

bool Leg::atTarget() {
  return (abs(target_pos.x - current_pos.x) < 2.0 &&
          abs(target_pos.y - current_pos.y) < 2.0 &&
          abs(target_pos.z - current_pos.z) < 2.0);
}

void Leg::moveToLocal(Point local_pos) {
  LegAngles angles = inverseKinematics(local_pos);
  if (!isnan(angles.coxa)) {
    LegAngles servo_angles = convertToServoAngles(angles);
    coxa_servo.write(servo_angles.coxa);
    femur_servo.write(servo_angles.femur);
    tibia_servo.write(servo_angles.tibia);
  }
}

LegAngles Leg::inverseKinematics(Point localPos) {
  LegAngles angles = {0, 0, 0};
  float l1 = COXA_LEN, l2 = FEMUR_LEN, l3 = TIBIA_LEN;

  // IK Logic
  angles.coxa = degrees(atan2(localPos.y, localPos.x));

  float d = sqrt(pow(localPos.x, 2) + pow(localPos.y, 2));
  float D = sqrt(pow(d - l1, 2) + pow(localPos.z, 2));

  if (D > (l2 + l3))
    return {NAN, NAN, NAN};

  float alpha1 = acos((pow(l2, 2) + pow(l3, 2) - pow(D, 2)) / (2 * l2 * l3));
  float beta = acos((pow(l2, 2) + pow(D, 2) - pow(l3, 2)) / (2 * l2 * D));
  float alpha2 = atan2(localPos.z, d - l1);

  angles.tibia = 180.0 - degrees(alpha1);
  angles.femur = degrees(alpha2 + beta);

  return angles;
}

LegAngles Leg::convertToServoAngles(LegAngles kinematicAngles) {
  LegAngles servoAngles;

  // Servo Mapping based on Leg Index (FR=0, FL=1, BR=2, BL=3)
  if (_leg_index == 0) { // FR
    servoAngles.coxa = kinematicAngles.coxa + 90.0;
    servoAngles.femur = 90.0 - kinematicAngles.femur;
    servoAngles.tibia = 180.0 - kinematicAngles.tibia;
  } else if (_leg_index == 1) { // FL
    servoAngles.coxa = abs(kinematicAngles.coxa) - 90.0;
    servoAngles.femur = 90.0 + kinematicAngles.femur;
    servoAngles.tibia = kinematicAngles.tibia;
  } else if (_leg_index == 2) { // BR
    servoAngles.coxa = 90.0 - abs(kinematicAngles.coxa);
    servoAngles.femur = 90.0 + kinematicAngles.femur;
    servoAngles.tibia = kinematicAngles.tibia;
  } else if (_leg_index == 3) { // BL
    servoAngles.coxa = 90.0 + (180.0 - abs(kinematicAngles.coxa));
    servoAngles.femur = 90.0 - kinematicAngles.femur;
    servoAngles.tibia = 180.0 - kinematicAngles.tibia;
  }

  return servoAngles;
}

// ==========================================
// GaitEngine.cpp Content
// ==========================================

GaitEngine::GaitEngine() {
  // Initialize Legs with specific pins and mounting positions
  // FR: {4, 2, 3}, Pos {35.5, 35.25}
  legs[0] = new Leg(4, 2, 3, 35.5, 35.25, 0);

  // FL: {10, 8, 9}, Pos {-35.5, 35.25}
  legs[1] = new Leg(10, 8, 9, -35.5, 35.25, 1);

  // BR: {7, 5, 6}, Pos {35.5, -35.25}
  legs[2] = new Leg(7, 5, 6, 35.5, -35.25, 2);

  // BL: {13, 11, 12}, Pos {-35.5, -35.25}
  legs[3] = new Leg(13, 11, 12, -35.5, -35.25, 3);

  currentState = STOPPED;
}

void GaitEngine::init() {
  for (int i = 0; i < 4; i++) {
    legs[i]->init();
  }
}

void GaitEngine::commandMoveForward() {
  if (currentState == STOPPED) {
    currentState = LIFT_FR;
    stateStartTime = millis();

    // Preset Legs to "Start of Cycle" position for smoothness?
    // Ideally, we start from neutral.
    // If we start from Neutral:
    // FR Swing -> Neutral + L/2.
    // Others Stance -> Neutral - L/4.
    // This is a fine start.
  }
}

void GaitEngine::commandStop() {
  currentState = STOPPED;
  // Reset all to Neutral
  for (int i = 0; i < 4; i++) {
    Point neu = legs[i]->getLocalNeutralPos();
    Point mount = legs[i]->getMountingPos();
    Point goal = {mount.x + neu.x, mount.y + neu.y, mount.z + neu.z};
    legs[i]->setTarget(goal);
  }
}

void GaitEngine::update() {
  // Time Delta
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  if (dt > 0.1)
    dt = 0.02; // Safety cap
  lastUpdate = now;

  // Run Leg Updates
  for (int i = 0; i < 4; i++) {
    legs[i]->update(dt);
  }

  // State Machine
  switch (currentState) {
  case STOPPED:
    break;

  case LIFT_FR:
    if (runPhase(0))
      currentState = LIFT_BL; // Next Leg in sequence
    break;

  case LIFT_BL:
    if (runPhase(3))
      currentState = LIFT_FL;
    break;

  case LIFT_FL:
    if (runPhase(1))
      currentState = LIFT_BR;
    break;

  case LIFT_BR:
    if (runPhase(2))
      currentState = LIFT_FR; // Loop
    break;

  case MOVING_BODY: // Legacy
    break;
  }
}

// Executes one phase where 'swing_leg_index' moves forward (Swing)
// and all other legs move backward (Stance).
// Returns true when phase is complete.
bool GaitEngine::runPhase(int swing_leg_index) {

  Leg *swingLeg = legs[swing_leg_index];

  // 1. Calculate Targets

  // -- Swing Leg Target --
  // Goal: Neutral Y + (L/2). X = Neutral X.

  Point neu = swingLeg->getLocalNeutralPos();
  Point mount = swingLeg->getMountingPos();

  // Convert Local Neutral to Global
  Point swingTargetGlobal;
  swingTargetGlobal.x = mount.x + neu.x;
  swingTargetGlobal.y = mount.y + neu.y + (STRIDE_LENGTH / 2.0);
  swingTargetGlobal.z = mount.z + neu.z; // Ground level Z (-60)

  // Lift Logic: If far from target Y, Lift Z.
  // Simple "Bang-Bang" lift
  Point currentPos = swingLeg->getCurrentPos();
  float distToTarget = sqrt(pow(currentPos.x - swingTargetGlobal.x, 2) +
                            pow(currentPos.y - swingTargetGlobal.y, 2));

  Point actualTarget = swingTargetGlobal;
  if (distToTarget > 5.0) {
    // Mid-Air point
    actualTarget.z += LIFT_HEIGHT;
  } else {
    // Descend
    actualTarget.z = swingTargetGlobal.z;
  }

  swingLeg->setTarget(actualTarget);

  // -- Stance Legs Targets --
  // All other legs move backward by L/4 relative to BODY (so Leg Y decreases?
  // No) If Body moves Forward (+Y), Leg must move Backward (-Y) relative to
  // Body. Wait, if robot is facing +Y, legs push -Y. Yes. "Move Y backward by
  // L/4".

  // Since we want controlled speed, we don't just set the target to "End
  // Point". We incrementally move the target so the leg follows a smooth path.
  // Speed = (L/4) / PhaseDuration?
  // Or just set target to (Current - L/4) and let Leg::update handle speed?
  // If Leg::update uses MOVE_SPEED (150mm/s), that's fast.
  // L/4 = 10mm.
  // If we set target to -10mm, leg arrives in 0.06s.
  // But swing takes longer.
  // We want Stance legs to move *during* the swing.
  // So we should check if Swing is Done.

  // Simple Logic:
  // Set Stance Targets to "Current - delta" ONLY if they are "idle"?
  // No, we need continuous motion.

  // Let's use the "Target Endpoint" approach.
  // For the active phase, the stance target is "Start of Phase - L/4".
  // Note: This requires knowing "Start of Phase" position.
  // But "Current Target" is effectively where we want to satisfy.

  // Let's just set the Stance Targets to (Neutral - Something)??
  // No, Stance is relative.

  // Correct approach for simple state machine:
  // IF starting the phase (one-shot):
  //   Set Swing Final Target.
  //   Set Stance Final Targets = Current_Target - (L/4).
  // THEN:
  //   Wait for Swing Leg to reach Final Target.
  //   (Stance legs will arrive early if they are faster, and wait. That's
  //   acceptable for now).

  static int last_swing_index = -1;
  static unsigned long phase_start_time = 0;

  if (last_swing_index != swing_leg_index) {
    // INIT PHASE
    last_swing_index = swing_leg_index;
    phase_start_time = millis();

    // Set Stance Targets
    for (int i = 0; i < 4; i++) {
      if (i == swing_leg_index)
        continue;

      Point t =
          legs[i]->getTargetPos(); // Use current target to avoid drift from lag
      t.y -= (STRIDE_LENGTH / 4.0); // Move Back 10mm
      legs[i]->setTarget(t);
    }
  }

  // Check if Swing Leg is Done (At Ground Target)
  if (swingLeg->atTarget()) {
    // Ensure we are at the "Down" position, not just "Lift" mid-point.
    // Check if current target Z is close to the ground Z (swingTargetGlobal.z)
    if (abs(swingLeg->getTargetPos().z - swingTargetGlobal.z) < 1.0) {
      last_swing_index = -1; // Reset for next call
      return true;
    }
  }

  return false;
}

// ==========================================
// Main Arduino Sketch
// ==========================================

GaitEngine gaitEngine;

void update_gait_isr() { gaitEngine.update(); }

void setup() {
  Serial.begin(9600);
  Serial.println("Quadruped Gait Engine Initializing...");

  gaitEngine.init();

  // Set up timer interrupt for 50Hz (20ms)
  FlexiTimer2::set(20, update_gait_isr);
  FlexiTimer2::start();

  Serial.println("Ready. Type 'w' to walk, 's' to stop.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'w') {
      Serial.println("Walking...");
      gaitEngine.commandMoveForward();
    } else if (cmd == 's') {
      Serial.println("Stopping...");
      gaitEngine.commandStop();
    }
  }
}
