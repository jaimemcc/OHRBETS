# OHRBehavior Library - Quick Reference

## Class Overview

| Class | Purpose | Key Methods |
|-------|---------|-------------|
| `LickDetector` | Detect licks via capacitive or voltage sensor | `begin()`, `checkLick()`, `openLickGate()` |
| `ServoController` | Control brake, retract, and radial servos | `engageBrake()`, `extendSpout()`, `rotateToSpout()` |
| `SolenoidController` | Manage liquid reward delivery | `openSolenoid()`, `closeAll()` |
| `RotaryEncoder` | Read rotary encoder for operant tasks | `getRightRotations()`, `getPosition()` |
| `SessionManager` | Handle timing and logging | `update()`, `getTimestamp()`, `logEvent()` |
| `TTLManager` | Send TTL pulses for external timestamps | `addPin()`, `sendPulse()` |

## Common Usage Patterns

### Initialize Session
```cpp
SessionManager session;
session.begin(600000); // 10 minutes
session.waitForStart(); // Wait for serial command
```

### Setup Lick Detection
```cpp
LickDetector lickDetector(0, 21); // cap sensor, pin 21
lickDetector.begin();
lickDetector.setThresholds(6, 2); // Optional: adjust sensitivity
```

### Configure Servos
```cpp
ServoController servos;
servos.initBrake(10, 15, 0); // pin, engaged_deg, disengaged_deg
servos.initRetract(9, 75, extendedDegs, numSpouts);
servos.initRadial(11, radialDegs, numSpouts);
```

### Setup Solenoids
```cpp
byte pins[] = {4, 5, 6, 7, 8};
byte durations[] = {30, 30, 30, 30, 30};
SolenoidController solenoids(5);
solenoids.begin(pins, durations);
```

### Setup Rotary Encoder (for operant tasks)
```cpp
RotaryEncoder encoder(3, 2, 32); // pinA, pinB, resolution
encoder.begin();

// In ISR functions:
void ISR_A() { encoder.handleInterruptA(); }
void ISR_B() { encoder.handleInterruptB(); }
attachInterrupt(digitalPinToInterrupt(3), ISR_A, RISING);
attachInterrupt(digitalPinToInterrupt(2), ISR_B, RISING);
```

### Main Loop Structure
```cpp
void loop() {
  // Update timestamp and controllers
  session.update();
  unsigned long ts = session.getTimestamp();
  servos.update(ts);
  solenoids.update(ts);
  ttls.update(ts);
  
  // Your experiment logic here
  
  // Check for session end
  if (session.isSessionComplete()) {
    endSession();
  }
}
```

### Lick Detection and Reward
```cpp
byte lick = lickDetector.checkLick(currentSpout);
if (lick > 0 && lickDetector.isLickGateOpen()) {
  session.logEvent(30 + lick, ts);
  solenoids.openSolenoid(currentSpout, ts);
  lickDetector.closeLickGate();
  // Set gate reopen time...
}
```

### Servo Operations
```cpp
// Brake control
servos.engageBrake(ts);
servos.disengageBrake(ts);

// Spout control
servos.extendSpout(spoutIndex, ts);
servos.retractSpout(ts);

// Radial positioning
servos.rotateToSpout(spoutIndex, ts);
servos.rotateToSweep(77, ts); // Neutral position
```

### Event Logging
```cpp
session.logEvent(1, ts);           // Log session start
session.logParameter(100, 600000); // Log session duration
session.logEvent(30 + spout, ts);  // Log lick on spout
```

### TTL Pulses
```cpp
TTLManager ttls;
ttls.addPin(52); // Solenoid TTL
ttls.addPin(50); // Rotation TTL
ttls.sendPulse(0, ts, 5); // Send 5ms pulse on pin index 0
```

### Rotary Encoder Reading
```cpp
int rightRot = encoder.getRightRotations();
int leftRot = encoder.getLeftRotations();
int position = encoder.getPosition();

if (rightRot > 0) {
  session.logEvent(81, ts); // Log right rotation
}
```

## Common Parameters

### Detection Circuits
- `0` = Capacitive touch sensor (MPR121)
- `1` = Voltage-based sensor

### Event ID Standards
```cpp
1   - Session start
0   - Session end
11  - Brake engaged
12  - Brake disengaged
13  - Spout extended
15  - Spout retracted
30+n - Lick on spout n
40+n - Solenoid open for spout n
50+n - Pump on for spout n
71  - Left rotation
81  - Right rotation
82  - Criteria reached
99  - Session ended (time)
127 - Current spout
130 - Radial position moved
```

## Timing Guidelines

- Servo detachment: 250ms after movement
- Solenoid duration: 20-40ms (calibrate for ~1.5µL)
- Lick latency minimum: 50ms
- TTL pulse duration: 5ms

## Migration Checklist

When converting an existing program:

- [ ] Replace includes with `#include <OHRBehavior.h>`
- [ ] Replace `Adafruit_MPR121 cap` with `LickDetector`
- [ ] Replace servo objects with `ServoController`
- [ ] Replace solenoid arrays with `SolenoidController`
- [ ] Replace timestamp code with `SessionManager`
- [ ] Move pin definitions to setup arrays
- [ ] Replace servo attach/write/detach with servo methods
- [ ] Replace manual TTL code with `TTLManager`
- [ ] Test with existing Python analysis scripts

## Tips

1. **Always call `update()` methods** in the main loop for servos, solenoids, and TTLs
2. **Use 0-indexed arrays** internally but log 1-indexed spout numbers
3. **Check lick gate** before responding to licks
4. **Session timing**: Call `session.update()` at start of each loop iteration
5. **Servo power**: Library automatically detaches servos to save power
6. **Interrupt conflicts**: If using rotary encoder, be careful with servo timing
