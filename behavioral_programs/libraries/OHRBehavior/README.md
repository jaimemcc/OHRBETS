# OHRBehavior Arduino Library

An Arduino library for OHRBETS behavioral experimental programs. This library consolidates common functionality across multiple behavioral paradigms including free access, operant conditioning, retractable spout, multi-spout, and real-time place testing (RTPT) programs.

## Features

The library provides six main classes that handle different aspects of behavioral experiments:

### 1. **LickDetector**
Handles lick detection using either capacitive touch sensors (MPR121) or voltage-based sensors.
- Automatic touch onset detection
- Lick gating to prevent rapid repeated detections
- Support for multiple spouts
- Configurable thresholds

### 2. **ServoController**
Manages three types of servos commonly used in behavioral setups:
- **Brake servo**: Controls access to the response wheel
- **Retractable spout servo**: Extends/retracts the spout
- **Radial servo**: Rotates multi-spout head to different positions
- Automatic servo detachment after movement to reduce power consumption

### 3. **SolenoidController**
Controls liquid reward delivery through solenoids:
- Independent timing for each spout
- Automatic closure after calibrated duration
- Support for multiple spouts with different durations

### 4. **RotaryEncoder**
Reads rotary encoder for operant responses:
- Interrupt-based reading for accuracy
- Separate tracking of left/right rotations
- Configurable resolution (downsampling)
- Position tracking

### 5. **SessionManager**
Handles session timing and event logging:
- Timestamp generation
- Serial communication for event logging
- Session duration tracking
- Parameter logging

### 6. **TTLManager**
Manages TTL output pulses for external time-stamping:
- Multiple TTL channels
- Automatic pulse termination
- Configurable pulse duration

## Installation

### Arduino IDE
1. Download this repository as a ZIP file
2. In Arduino IDE, go to Sketch → Include Library → Add .ZIP Library
3. Select the downloaded ZIP file
4. The library will be installed in your Arduino libraries folder

### Manual Installation
1. Navigate to your Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries`
   - Mac: `~/Documents/Arduino/libraries`
   - Linux: `~/Arduino/libraries`
2. Copy the `OHRBehavior` folder into the libraries folder
3. Restart Arduino IDE

## Dependencies

This library requires:
- **Adafruit_MPR121** - For capacitive touch sensing
- **Servo** - For servo motor control (included with Arduino IDE)
- **Wire** - For I2C communication (included with Arduino IDE)

Install Adafruit_MPR121 through the Arduino Library Manager:
1. Sketch → Include Library → Manage Libraries
2. Search for "Adafruit MPR121"
3. Install the library

## Usage

### Basic Example

```cpp
#include <OHRBehavior.h>

// Create library objects
SessionManager session;
LickDetector lickDetector(0, 21); // 0=cap sensor, pin 21
ServoController servos;
SolenoidController solenoids(5); // 5 spouts

void setup() {
  Serial.begin(115200);
  
  // Initialize components
  session.begin(600000); // 10 minute session
  lickDetector.begin();
  
  // Configure servos
  servos.initBrake(10, 15, 0);
  servos.initRetract(9, 75, extendedDegs, 5);
  servos.initRadial(11, radialDegs, 5);
  
  // Configure solenoids
  solenoids.begin(solPins, solDurations);
}

void loop() {
  session.update();
  unsigned long ts = session.getTimestamp();
  
  // Update all controllers
  servos.update(ts);
  solenoids.update(ts);
  
  // Check for licks
  byte lick = lickDetector.checkLick(currentSpout);
  if (lick > 0) {
    solenoids.openSolenoid(currentSpout, ts);
  }
  
  if (session.isSessionComplete()) {
    endSession();
  }
}
```

See the `examples/BasicUsage` folder for a complete working example.

## Migrating Existing Programs

To migrate your existing behavioral programs to use this library:

1. **Replace common includes** with the library:
   ```cpp
   // Old
   #include <Wire.h>
   #include "Adafruit_MPR121.h"
   #include <Servo.h>
   
   // New
   #include <OHRBehavior.h>
   ```

2. **Replace manual object creation** with library classes:
   ```cpp
   // Old
   Adafruit_MPR121 cap = Adafruit_MPR121();
   Servo servo_brake;
   
   // New
   LickDetector lickDetector(0, 21);
   ServoController servos;
   ```

3. **Replace repetitive code** with library methods:
   ```cpp
   // Old
   servo_brake.attach(pinServo_brake);
   servo_brake.write(servo_brake_engaged_deg);
   Serial.print(11); Serial.print(" "); Serial.println(ts);
   delay(250);
   servo_brake.detach();
   
   // New
   servos.engageBrake(ts);
   ```

## Library Structure

```
OHRBehavior/
├── examples/
│   └── BasicUsage/
│       └── BasicUsage.ino
├── src/
│   ├── OHRBehavior.h
│   └── OHRBehavior.cpp
├── keywords.txt
├── library.properties
└── README.md
```

## Event IDs

The library uses standard event IDs for logging (compatible with existing analysis scripts):

- `1` - Session start
- `0` - Session end
- `11` - Brake engaged
- `12` - Brake disengaged
- `13` - Spout extended
- `15` - Spout retracted
- `30+n` - Lick detected on spout n
- `40+n` - Solenoid opened for spout n
- `81` - Right rotation
- `71` - Left rotation
- `130` - Radial servo moved

## Benefits

Using this library provides several advantages:

1. **Code Reusability**: Write once, use across multiple programs
2. **Reduced Errors**: Tested, reliable code reduces bugs
3. **Easier Maintenance**: Update library once, all programs benefit
4. **Cleaner Code**: Focus on experiment logic, not hardware details
5. **Consistency**: Standardized event logging across experiments
6. **Documentation**: Well-documented API makes code easier to understand

## Compatibility

This library is designed to be compatible with existing OHRBETS programs:
- ✓ beh_freeaccess
- ✓ beh_multispout_briefaccess
- ✓ beh_operant
- ✓ beh_retractablespout
- ✓ beh_rtpt

All event IDs and serial output formats match the original programs, ensuring compatibility with existing Python analysis scripts.

## Contributing

To contribute to this library:
1. Test changes with multiple behavioral programs
2. Maintain backward compatibility with existing programs
3. Update documentation and examples
4. Follow Arduino library guidelines

## License

This library is part of the OHRBETS project.

## Support

For issues or questions, please refer to the main OHRBETS repository.
