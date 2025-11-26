/*
  OHRBehavior.cpp - Implementation of Arduino library for OHRBETS behavioral programs
*/

#include "OHRBehavior.h"

// ==================== LICK DETECTOR IMPLEMENTATION ====================
LickDetector::LickDetector(byte detection_circuit, byte pin_lickometer) {
  _detection_circuit = detection_circuit;
  _pin_lickometer = pin_lickometer;
  _lasttouched = 0;
  _currtouched = 0;
  _pinLickometer_state = false;
  _pinLickometer_state_previous = false;
  _lick_gate = true;
}

bool LickDetector::begin() {
  if (_detection_circuit == 0) { // Capacitive sensor
    if (!_cap.begin(0x5A)) {
      Serial.println("MPR121 not detected!");
      return false;
    }
    _cap.setThresholds(6, 2); // Default thresholds
    delay(50);
    return true;
  } else { // Voltage sensor
    pinMode(_pin_lickometer, INPUT);
    return true;
  }
}

void LickDetector::setThresholds(byte touch, byte release) {
  if (_detection_circuit == 0) {
    _cap.setThresholds(touch, release);
  }
}

uint16_t LickDetector::getFilteredData(byte electrode) {
  if (_detection_circuit == 0) {
    return _cap.filteredData(electrode);
  }
  return 0;
}

byte LickDetector::checkLick(byte current_spout) {
  if (_detection_circuit == 0) { // Capacitive sensor
    _currtouched = _cap.touched();
    
    // Check for touch onset on current spout
    if ((_currtouched & _BV(current_spout)) && !(_lasttouched & _BV(current_spout))) {
      _lasttouched = _currtouched;
      return current_spout;
    }
    
    _lasttouched = _currtouched;
    return 0;
    
  } else { // Voltage sensor
    _pinLickometer_state = digitalRead(_pin_lickometer);
    
    if (_pinLickometer_state > _pinLickometer_state_previous) {
      _pinLickometer_state_previous = _pinLickometer_state;
      return current_spout + 1;
    }
    
    _pinLickometer_state_previous = _pinLickometer_state;
    return 0;
  }
}

bool LickDetector::isLickGateOpen() {
  return _lick_gate;
}

void LickDetector::closeLickGate() {
  _lick_gate = false;
}

void LickDetector::openLickGate() {
  _lick_gate = true;
}

// ==================== SERVO CONTROLLER IMPLEMENTATION ====================
ServoController::ServoController() {
  _detach_brake_ts = 0;
  _detach_retract_ts = 0;
  _detach_radial_ts = 0;
}

void ServoController::initBrake(byte pin, byte engaged_deg, byte disengaged_deg) {
  _pin_brake = pin;
  _brake_engaged_deg = engaged_deg;
  _brake_disengaged_deg = disengaged_deg;
  pinMode(pin, OUTPUT);
}

void ServoController::initRetract(byte pin, byte retracted_deg, byte* extended_degs, byte num_spouts) {
  _pin_retract = pin;
  _retract_retracted_deg = retracted_deg;
  _retract_extended_degs = extended_degs;
  pinMode(pin, OUTPUT);
}

void ServoController::initRadial(byte pin, byte* radial_degs, byte num_spouts) {
  _pin_radial = pin;
  _radial_degs = radial_degs;
  pinMode(pin, OUTPUT);
}

void ServoController::engageBrake(unsigned long ts) {
  _servo_brake.attach(_pin_brake);
  _servo_brake.write(_brake_engaged_deg);
  Serial.print(11); Serial.print(" "); Serial.println(ts);
  _detach_brake_ts = ts + DETACH_SERVO_STEP;
}

void ServoController::disengageBrake(unsigned long ts) {
  _servo_brake.attach(_pin_brake);
  _servo_brake.write(_brake_disengaged_deg);
  Serial.print(12); Serial.print(" "); Serial.println(ts);
  _detach_brake_ts = ts + DETACH_SERVO_STEP;
}

void ServoController::extendSpout(byte spout_index, unsigned long ts) {
  _servo_retract.attach(_pin_retract);
  _servo_retract.write(_retract_extended_degs[spout_index]);
  Serial.print(13); Serial.print(" "); Serial.println(ts);
  _detach_retract_ts = ts + DETACH_SERVO_STEP;
}

void ServoController::retractSpout(unsigned long ts) {
  _servo_retract.attach(_pin_retract);
  _servo_retract.write(_retract_retracted_deg);
  Serial.print(15); Serial.print(" "); Serial.println(ts);
  _detach_retract_ts = ts + DETACH_SERVO_STEP;
}

void ServoController::rotateToSpout(byte spout_index, unsigned long ts) {
  _servo_radial.attach(_pin_radial);
  _servo_radial.write(_radial_degs[spout_index]);
  Serial.print(130); Serial.print(" "); Serial.println(ts);
  _detach_radial_ts = ts + DETACH_SERVO_STEP;
}

void ServoController::rotateToSweep(byte sweep_deg, unsigned long ts) {
  _servo_radial.attach(_pin_radial);
  _servo_radial.write(sweep_deg);
  _detach_radial_ts = ts + DETACH_SERVO_STEP;
}

void ServoController::update(unsigned long ts) {
  // Detach servos after they've had time to move
  if (ts >= _detach_brake_ts && _detach_brake_ts != 0) {
    _servo_brake.detach();
    _detach_brake_ts = 0;
  }
  
  if (ts >= _detach_retract_ts && _detach_retract_ts != 0) {
    _servo_retract.detach();
    _detach_retract_ts = 0;
  }
  
  if (ts >= _detach_radial_ts && _detach_radial_ts != 0) {
    _servo_radial.detach();
    _detach_radial_ts = 0;
  }
}

// ==================== SOLENOID CONTROLLER IMPLEMENTATION ====================
SolenoidController::SolenoidController(byte num_spouts) {
  _num_spouts = num_spouts;
  _ts_sol_offset = 0;
  _active_spout = 0;
}

void SolenoidController::begin(byte* pins, byte* durations) {
  _pins = pins;
  _durations = durations;
  
  for (byte i = 0; i < _num_spouts; i++) {
    pinMode(_pins[i], OUTPUT);
    digitalWrite(_pins[i], LOW);
  }
}

void SolenoidController::openSolenoid(byte spout_index, unsigned long ts) {
  digitalWrite(_pins[spout_index], HIGH);
  Serial.print(40 + spout_index + 1); Serial.print(" "); Serial.println(ts);
  _ts_sol_offset = ts + _durations[spout_index];
  _active_spout = spout_index;
}

void SolenoidController::update(unsigned long ts) {
  if (ts >= _ts_sol_offset && _ts_sol_offset != 0) {
    digitalWrite(_pins[_active_spout], LOW);
    Serial.print(14); Serial.print(" "); Serial.println(ts);
    _ts_sol_offset = 0;
  }
}

void SolenoidController::closeAll() {
  for (byte i = 0; i < _num_spouts; i++) {
    digitalWrite(_pins[i], LOW);
  }
}

byte SolenoidController::getDuration(byte spout_index) {
  return _durations[spout_index];
}

// ==================== ROTARY ENCODER IMPLEMENTATION ====================
RotaryEncoder::RotaryEncoder(byte pinA, byte pinB, int resolution) {
  _pinA = pinA;
  _pinB = pinB;
  _resolution = resolution;
  _rotation_right_counter = 0;
  _rotation_left_counter = 0;
  _rotation_right = 0;
  _rotation_left = 0;
  _rotation_position = 0;
  _rotation_right_flag = false;
  _rotation_left_flag = false;
  _aFlag = 0;
  _bFlag = 0;
}

void RotaryEncoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  
  _maskA = digitalPinToBitMask(_pinA);
  _maskB = digitalPinToBitMask(_pinB);
  _maskAB = _maskA | _maskB;
  _port = portInputRegister(digitalPinToPort(_pinA));
}

void RotaryEncoder::handleInterruptA() {
  noInterrupts();
  volatile byte reading = *_port & _maskAB;
  if (reading == _maskAB && _bFlag) {
    _rotation_right_flag = true;
    _bFlag = 0;
    _aFlag = 0;
  } else if (reading == _maskA) {
    _aFlag = 1;
  }
  interrupts();
}

void RotaryEncoder::handleInterruptB() {
  noInterrupts();
  volatile byte reading = *_port & _maskAB;
  if ((reading == _maskAB) && _aFlag) {
    _rotation_left_flag = true;
    _bFlag = 0;
    _aFlag = 0;
  } else if (reading == _maskB) {
    _bFlag = 1;
  }
  interrupts();
}

int RotaryEncoder::getRightRotations() {
  if (_rotation_right_flag) {
    _rotation_right_counter++;
    _rotation_right_flag = false;
    
    if (_rotation_right_counter >= _resolution) {
      _rotation_right++;
      _rotation_position++;
      _rotation_right_counter = 0;
      return _rotation_right;
    }
  }
  return 0;
}

int RotaryEncoder::getLeftRotations() {
  if (_rotation_left_flag) {
    _rotation_left_counter++;
    _rotation_left_flag = false;
    
    if (_rotation_left_counter >= _resolution) {
      _rotation_left++;
      _rotation_position--;
      _rotation_left_counter = 0;
      return _rotation_left;
    }
  }
  return 0;
}

int RotaryEncoder::getPosition() {
  return _rotation_position;
}

void RotaryEncoder::resetCounters() {
  _rotation_right = 0;
  _rotation_left = 0;
  _rotation_right_counter = 0;
  _rotation_left_counter = 0;
}

// ==================== SESSION MANAGER IMPLEMENTATION ====================
SessionManager::SessionManager() {
  _ts_start = 0;
  _ts = 0;
  _session_end_ts = 0;
  _session_duration = 0;
}

void SessionManager::begin(unsigned long duration_ms) {
  _session_duration = duration_ms;
  _ts_start = millis();
}

void SessionManager::update() {
  _ts = millis() - _ts_start;
}

unsigned long SessionManager::getTimestamp() {
  return _ts;
}

bool SessionManager::isSessionComplete() {
  return (_session_end_ts != 0 && _ts > _session_end_ts);
}

void SessionManager::waitForStart() {
  while (Serial.available() <= 0) {
    // Wait for serial input to start session
  }
  delay(100);
}

void SessionManager::logEvent(int event_id, unsigned long ts) {
  Serial.print(event_id);
  Serial.print(" ");
  Serial.println(ts);
}

void SessionManager::logParameter(int param_id, unsigned long value) {
  Serial.print(param_id);
  Serial.print(" ");
  Serial.println(value);
}

void SessionManager::logParameter(int param_id, double value) {
  Serial.print(param_id);
  Serial.print(" ");
  Serial.println(value);
}

void SessionManager::setEndTime(unsigned long end_ts) {
  _session_end_ts = end_ts;
}

// ==================== TTL MANAGER IMPLEMENTATION ====================
TTLManager::TTLManager() {
  _num_pins = 0;
  for (byte i = 0; i < MAX_TTLS; i++) {
    _ttl_off_times[i] = 0;
  }
}

void TTLManager::addPin(byte pin) {
  if (_num_pins < MAX_TTLS) {
    _pins[_num_pins] = pin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    _num_pins++;
  }
}

void TTLManager::sendPulse(byte pin_index, unsigned long ts, int duration_ms) {
  if (pin_index < _num_pins) {
    digitalWrite(_pins[pin_index], HIGH);
    _ttl_off_times[pin_index] = ts + duration_ms;
  }
}

void TTLManager::update(unsigned long ts) {
  for (byte i = 0; i < _num_pins; i++) {
    if (ts >= _ttl_off_times[i] && _ttl_off_times[i] != 0) {
      digitalWrite(_pins[i], LOW);
      _ttl_off_times[i] = 0;
    }
  }
}
