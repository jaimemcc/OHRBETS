/*
  OHRBehavior.h - Arduino library for OHRBETS behavioral programs
  
  This library consolidates common functionality across behavioral programs including:
  - Capacitive touch sensing (lick detection)
  - Servo control (brake, retractable spout, radial positioning)
  - Solenoid control
  - Rotary encoder reading
  - Serial event logging
  - Session timing
  
  Created for the OHRBETS project
*/

#ifndef OHRBehavior_h
#define OHRBehavior_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <Servo.h>

// ==================== LICK DETECTOR CLASS ====================
class LickDetector {
  public:
    // Constructor
    LickDetector(byte detection_circuit, byte pin_lickometer);
    
    // Initialize the detector
    bool begin();
    
    // Check for lick onset, returns spout number (1-indexed) or 0 if no lick
    byte checkLick(byte current_spout);
    
    // Set capacitive sensor thresholds
    void setThresholds(byte touch, byte release);
    
    // Get filtered data from cap sensor
    uint16_t getFilteredData(byte electrode);
    
    // Check if lick gate allows detection
    bool isLickGateOpen();
    void closeLickGate();
    void openLickGate();
    
  private:
    byte _detection_circuit; // 0: cap sensor, 1: voltage sensor
    byte _pin_lickometer;
    Adafruit_MPR121 _cap;
    uint16_t _lasttouched;
    uint16_t _currtouched;
    boolean _pinLickometer_state;
    boolean _pinLickometer_state_previous;
    boolean _lick_gate;
};

// ==================== SERVO CONTROLLER CLASS ====================
class ServoController {
  public:
    // Constructor
    ServoController();
    
    // Initialize servos
    void initBrake(byte pin, byte engaged_deg, byte disengaged_deg);
    void initRetract(byte pin, byte retracted_deg, byte* extended_degs, byte num_spouts);
    void initRadial(byte pin, byte* radial_degs, byte num_spouts);
    
    // Brake control
    void engageBrake(unsigned long ts);
    void disengageBrake(unsigned long ts);
    
    // Retractable spout control
    void extendSpout(byte spout_index, unsigned long ts);
    void retractSpout(unsigned long ts);
    
    // Radial positioning control
    void rotateToSpout(byte spout_index, unsigned long ts);
    void rotateToSweep(byte sweep_deg, unsigned long ts);
    
    // Update function (call in loop to handle servo detachment)
    void update(unsigned long ts);
    
  private:
    Servo _servo_brake;
    Servo _servo_retract;
    Servo _servo_radial;
    
    byte _pin_brake;
    byte _pin_retract;
    byte _pin_radial;
    
    byte _brake_engaged_deg;
    byte _brake_disengaged_deg;
    byte _retract_retracted_deg;
    byte* _retract_extended_degs;
    byte* _radial_degs;
    
    unsigned long _detach_brake_ts;
    unsigned long _detach_retract_ts;
    unsigned long _detach_radial_ts;
    
    static const int DETACH_SERVO_STEP = 250; // time to allow servo travel
};

// ==================== SOLENOID CONTROLLER CLASS ====================
class SolenoidController {
  public:
    // Constructor
    SolenoidController(byte num_spouts);
    
    // Initialize solenoids
    void begin(byte* pins, byte* durations);
    
    // Open solenoid for specific spout
    void openSolenoid(byte spout_index, unsigned long ts);
    
    // Update function (call in loop to handle closure)
    void update(unsigned long ts);
    
    // Close all solenoids
    void closeAll();
    
    // Get duration for specific spout
    byte getDuration(byte spout_index);
    
  private:
    byte _num_spouts;
    byte* _pins;
    byte* _durations;
    unsigned long _ts_sol_offset;
    byte _active_spout;
};

// ==================== ROTARY ENCODER CLASS ====================
class RotaryEncoder {
  public:
    // Constructor
    RotaryEncoder(byte pinA, byte pinB, int resolution);
    
    // Initialize encoder with interrupts
    void begin();
    
    // Get rotation counts
    int getRightRotations();
    int getLeftRotations();
    int getPosition();
    
    // Reset counters
    void resetCounters();
    
    // Handle interrupts (must be public for ISR)
    void handleInterruptA();
    void handleInterruptB();
    
  private:
    byte _pinA;
    byte _pinB;
    int _resolution;
    volatile int _rotation_right_counter;
    volatile int _rotation_left_counter;
    volatile int _rotation_right;
    volatile int _rotation_left;
    volatile int _rotation_position;
    volatile boolean _rotation_right_flag;
    volatile boolean _rotation_left_flag;
    volatile byte _aFlag;
    volatile byte _bFlag;
    uint8_t _maskA;
    uint8_t _maskB;
    uint8_t _maskAB;
    volatile uint8_t* _port;
};

// ==================== SESSION MANAGER CLASS ====================
class SessionManager {
  public:
    // Constructor
    SessionManager();
    
    // Initialize session
    void begin(unsigned long duration_ms);
    
    // Update timestamp
    void update();
    
    // Get current timestamp
    unsigned long getTimestamp();
    
    // Check if session should end
    bool isSessionComplete();
    
    // Wait for serial start command
    void waitForStart();
    
    // Log event (event_id, timestamp)
    void logEvent(int event_id, unsigned long ts);
    
    // Log parameter (param_id, value)
    void logParameter(int param_id, unsigned long value);
    void logParameter(int param_id, double value);
    
    // Set session end time
    void setEndTime(unsigned long end_ts);
    
  private:
    unsigned long _ts_start;
    unsigned long _ts;
    unsigned long _session_end_ts;
    unsigned long _session_duration;
};

// ==================== TTL MANAGER CLASS ====================
class TTLManager {
  public:
    // Constructor
    TTLManager();
    
    // Add TTL output pin
    void addPin(byte pin);
    
    // Send TTL pulse
    void sendPulse(byte pin_index, unsigned long ts, int duration_ms = 5);
    
    // Update function (call in loop to turn off pulses)
    void update(unsigned long ts);
    
  private:
    static const byte MAX_TTLS = 10;
    byte _pins[MAX_TTLS];
    unsigned long _ttl_off_times[MAX_TTLS];
    byte _num_pins;
};

#endif
