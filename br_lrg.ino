/*
  Single Leg Motion Controller with Gait Engine & FlexiTimer2

  This program uses the FlexiTimer2 library to control a single 3-servo robot
  leg smoothly between three predefined static poses. This version is
  intended for standard Arduino boards (like Uno, Nano, etc.).

  - The Gait Engine handles the interpolation (moving from A to B).
  - FlexiTimer2 runs the engine at a constant rate.
  - The main loop() acts as a sequencer to command new poses.
*/

#include <FlexiTimer2.h> // Include the timer library
#include <Servo.h>
#include <math.h>

// =============================================================================
// ## Part 1: Robot Geometry and Configuration
// =============================================================================

// --- Link Lengths (in mm) ---
const float COXA_LEN = 27.5;
const float FEMUR_LEN = 55.0;
const float TIBIA_LEN = 77.5;

// --- Data Structures ---
struct Point {
  float x, y, z;
};
struct LegAngles {
  float coxa, femur, tibia;
};

// --- Servo Pin Configuration ---
const int fr_servoPins[3] = {7, 5, 6}; // {coxa, femur, tibia}
Servo fr_servos[3];

// --- FR Leg Coxa Joint Position (relative to Body Center) ---
const Point fr_coxaPosition = {35.5, -35.25, 0};

// =============================================================================
// ## Part 2: Gait Engine Implementation for a Single Leg
// =============================================================================

// --- State Variables ---
// 'volatile' is crucial as these are modified by a timer interrupt
volatile Point fr_site_now;    // The leg's current, real-time coordinate
volatile Point fr_site_expect; // The leg's target destination coordinate
volatile Point fr_temp_speed;  // The small (dx, dy, dz) vector to add each tick

// --- Motion Parameters ---
const float MOVE_SPEED = 2.0; // Controls how fast the leg moves between poses

// --- Predefined Poses ---
Point pose_neutral = {90.5, -87.25, -60.0};
Point pose_crouch = {90.5, -107.25, -60.0};
Point pose_wave = {90.5, -87.25, -35.0};
Point poses[] = {pose_crouch, pose_wave, pose_neutral};
const int numPoses = 3;

// --- Sequencer State ---
int poseIndex = 0;
unsigned long timeOfArrival = 0;
const long POSE_HOLD_TIME = 1000; // Hold each pose for 3 seconds

// =============================================================================
// ## Part 3: Kinematics and Servo Control
// =============================================================================

LegAngles inverseKinematics(Point localPos) {
  LegAngles angles = {0, 0, 0};
  float l1 = COXA_LEN, l2 = FEMUR_LEN, l3 = TIBIA_LEN;
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

LegAngles convertToServoAngles(LegAngles kinematicAngles) {
  LegAngles servoAngles;
  servoAngles.coxa = 90 - abs(kinematicAngles.coxa);
  servoAngles.femur = 90.0 + kinematicAngles.femur;
  servoAngles.tibia = kinematicAngles.tibia;
  return servoAngles;
}

// =============================================================================
// ## Part 4: Main Robot Control Logic
// =============================================================================

// --- Target Setter Function (from the gait engine) ---
void set_fr_leg_target(Point targetPos) {
  float length_x = targetPos.x - fr_site_now.x;
  float length_y = targetPos.y - fr_site_now.y;
  float length_z = targetPos.z - fr_site_now.z;
  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  if (length < 0.01) { // Avoid division by zero for very small moves
    fr_temp_speed.x = 0;
    fr_temp_speed.y = 0;
    fr_temp_speed.z = 0;
  } else {
    fr_temp_speed.x = length_x / length * MOVE_SPEED;
    fr_temp_speed.y = length_y / length * MOVE_SPEED;
    fr_temp_speed.z = length_z / length * MOVE_SPEED;
  }
  fr_site_expect.x = targetPos.x;
  fr_site_expect.y = targetPos.y;
  fr_site_expect.z = targetPos.z;
}

// --- Helper function to check if the leg has reached its destination ---
bool is_fr_leg_at_target() {
  return fr_site_now.x == fr_site_expect.x &&
         fr_site_now.y == fr_site_expect.y && fr_site_now.z == fr_site_expect.z;
}

// --- Motion Engine (called by FlexiTimer2 every 20ms) ---
void update_fr_leg_motion() {
  // Update X coordinate
  if (abs(fr_site_now.x - fr_site_expect.x) >= abs(fr_temp_speed.x))
    fr_site_now.x += fr_temp_speed.x;
  else
    fr_site_now.x = fr_site_expect.x;

  // Update Y coordinate
  if (abs(fr_site_now.y - fr_site_expect.y) >= abs(fr_temp_speed.y))
    fr_site_now.y += fr_temp_speed.y;
  else
    fr_site_now.y = fr_site_expect.y;

  // Update Z coordinate
  if (abs(fr_site_now.z - fr_site_expect.z) >= abs(fr_temp_speed.z))
    fr_site_now.z += fr_temp_speed.z;
  else
    fr_site_now.z = fr_site_expect.z;

  // --- After updating position, command the servos ---
  Point localPos = {fr_site_now.x - fr_coxaPosition.x,
                    fr_site_now.y - fr_coxaPosition.y,
                    fr_site_now.z - fr_coxaPosition.z};

  LegAngles kinematic = inverseKinematics(localPos);
  if (!isnan(kinematic.coxa)) {
    LegAngles servoAngles = convertToServoAngles(kinematic);
    fr_servos[0].write(servoAngles.coxa);
    fr_servos[1].write(servoAngles.femur);
    fr_servos[2].write(servoAngles.tibia);
  }
}

// --- Setup Function (runs once at start) ---
void setup() {
  Serial.begin(9600);
  Serial.println("Single Leg Controller with Gait Engine Initializing...");

  // Attach servos to pins
  for (int i = 0; i < 3; i++) {
    fr_servos[i].attach(fr_servoPins[i]);
  }

  // --- Initialize the leg's starting position ---
  fr_site_now.x = pose_neutral.x;
  fr_site_now.y = pose_neutral.y;
  fr_site_now.z = pose_neutral.z;
  fr_site_expect.x = pose_neutral.x;
  fr_site_expect.y = pose_neutral.y;
  fr_site_expect.z = pose_neutral.z;

  Point localPos = {fr_site_now.x - fr_coxaPosition.x,
                    fr_site_now.y - fr_coxaPosition.y,
                    fr_site_now.z - fr_coxaPosition.z};
  LegAngles kinematic = inverseKinematics(localPos);
  LegAngles servoAngles = convertToServoAngles(kinematic);
  fr_servos[0].write(servoAngles.coxa);
  fr_servos[1].write(servoAngles.femur);
  fr_servos[2].write(servoAngles.tibia);

  delay(1000);
  Serial.println("Ready. Starting timer and sequencer...");

  // Start the motion engine heartbeat (50 times per second)
  FlexiTimer2::set(20, update_fr_leg_motion);
  FlexiTimer2::start();
}

// --- Main Loop (now acts as a non-blocking sequencer) ---
void loop() {
  // Check if the leg has arrived at its target pose
  if (is_fr_leg_at_target()) {

    // Check if it's the first time we've arrived at this pose
    if (timeOfArrival == 0) {
      timeOfArrival = millis(); // Record the time of arrival
    }

    // Check if we have held the pose for the required duration
    if (millis() - timeOfArrival > POSE_HOLD_TIME) {
      // It's time to move to the next pose in the sequence
      poseIndex = (poseIndex + 1) % numPoses;
      Point nextTarget = poses[poseIndex];

      if (poseIndex == 0)
        Serial.println("\nMoving to Crouch...");
      if (poseIndex == 1)
        Serial.println("Moving to Wave...");
      if (poseIndex == 2)
        Serial.println("Returning to Neutral Stance...");

      // Command the gait engine to start moving to the new target
      set_fr_leg_target(nextTarget);

      timeOfArrival = 0; // Reset the arrival timer
    }
  }
}
