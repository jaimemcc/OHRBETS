/*
  BasicUsage.ino - Example sketch showing basic usage of the OHRBehavior library
  
  This example demonstrates a simple free access task where licks trigger solenoid openings.
  It shows how to use the library's main components:
  - SessionManager for timing
  - LickDetector for capacitive touch sensing
  - ServoController for brake/spout control
  - SolenoidController for reward delivery
  
*/

#include <OHRBehavior.h>

// ==================== SESSION PARAMETERS ====================
const unsigned long SESSION_DURATION = 600000; // 10 minutes in ms
const byte LICK_DETECTION_CIRCUIT = 0; // 0: cap sensor, 1: voltage sensor
const byte NUM_SPOUTS = 5;
const byte CURRENT_SPOUT = 0; // 0-indexed (spout 1 = index 0)

// ==================== PIN DEFINITIONS ====================
const byte PIN_LICKOMETER = 21;
const byte PIN_SERVO_BRAKE = 10;
const byte PIN_SERVO_RETRACT = 9;
const byte PIN_SERVO_RADIAL = 11;

// Solenoid pins (one per spout)
byte pinSol[] = {4, 5, 6, 7, 8};

// Solenoid durations in ms (calibrated for ~1.5µL per delivery)
byte solDuration[] = {30, 30, 30, 30, 30};

// Servo degrees for extended position (one per spout)
byte servoRetractExtendedDegs[] = {1, 1, 1, 1, 1};

// Servo degrees for radial positions (one per spout)
byte servoRadialDegs[] = {12, 44, 70, 97, 123};

// ==================== LIBRARY OBJECTS ====================
SessionManager session;
LickDetector lickDetector(LICK_DETECTION_CIRCUIT, PIN_LICKOMETER);
ServoController servos;
SolenoidController solenoids(NUM_SPOUTS);
TTLManager ttls;

// ==================== TIMING VARIABLES ====================
const int TM_LICK_LATENCY_MIN = 50; // Minimum time between lick detections
unsigned long ts_lick_gate_open = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  
  // Initialize session manager
  session.begin(SESSION_DURATION);
  
  // Initialize lick detector
  if (!lickDetector.begin()) {
    Serial.println("Failed to initialize lick detector!");
    while(1);
  }
  
  // Initialize servo controller
  servos.initBrake(PIN_SERVO_BRAKE, 15, 0); // engaged=15deg, disengaged=0deg
  servos.initRetract(PIN_SERVO_RETRACT, 75, servoRetractExtendedDegs, NUM_SPOUTS); // retracted=75deg
  servos.initRadial(PIN_SERVO_RADIAL, servoRadialDegs, NUM_SPOUTS);
  
  // Initialize solenoid controller
  solenoids.begin(pinSol, solDuration);
  
  // Setup servos before session starts
  servos.engageBrake(0);
  delay(250);
  servos.rotateToSpout(CURRENT_SPOUT, 0);
  delay(250);
  servos.extendSpout(CURRENT_SPOUT, 0);
  delay(250);
  
  // Log session parameters
  session.logEvent(998, lickDetector.getFilteredData(1)); // Cap sensor baseline
  
  // Wait for serial command to start
  session.waitForStart();
  
  // Initialize timing
  session.begin(SESSION_DURATION);
  session.logEvent(1, session.getTimestamp()); // Log session start
  session.logParameter(100, SESSION_DURATION);
}

// ==================== MAIN LOOP ====================
void loop() {
  // Update timestamp
  session.update();
  unsigned long ts = session.getTimestamp();
  
  // Update all controllers
  servos.update(ts);
  solenoids.update(ts);
  ttls.update(ts);
  
  // Open lick gate if needed
  if (ts > ts_lick_gate_open && ts_lick_gate_open != 0) {
    lickDetector.openLickGate();
    ts_lick_gate_open = 0;
  }
  
  // Check for licks
  byte lick = lickDetector.checkLick(CURRENT_SPOUT);
  
  if (lick > 0 && lickDetector.isLickGateOpen()) {
    // Log lick event
    session.logEvent(30 + lick, ts);
    
    // Open solenoid for reward
    solenoids.openSolenoid(CURRENT_SPOUT, ts);
    
    // Close lick gate temporarily
    lickDetector.closeLickGate();
    
    // Calculate when to reopen lick gate
    byte sol_dur = solenoids.getDuration(CURRENT_SPOUT);
    if (TM_LICK_LATENCY_MIN > sol_dur) {
      ts_lick_gate_open = ts + TM_LICK_LATENCY_MIN;
    } else {
      ts_lick_gate_open = ts + sol_dur;
    }
  }
  
  // Check if session should end
  if (session.isSessionComplete()) {
    endSession();
  }
}

// ==================== END SESSION ====================
void endSession() {
  unsigned long ts = session.getTimestamp();
  
  // Close all solenoids
  solenoids.closeAll();
  
  // Retract spout
  servos.retractSpout(ts);
  delay(250);
  
  // Engage brake
  servos.engageBrake(ts);
  delay(250);
  
  // Log end of session
  session.logEvent(0, ts);
  
  // Stop program
  while(1) {}
}
