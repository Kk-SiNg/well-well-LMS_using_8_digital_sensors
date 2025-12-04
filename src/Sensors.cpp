/*
 * Sensors. cpp
 * 8-Channel RLS-08 - DIGITAL MODE
 * INVERTED LOGIC: 0 = White Line, 1 = Black Surface
 */

#include "Sensors.h"

Sensors::Sensors() {
    lastPosition = 0.0;
}

void Sensors::setup() {
    // Configure all sensor pins as digital inputs
    pinMode(SENSOR_PIN_1, INPUT);
    pinMode(SENSOR_PIN_2, INPUT);
    pinMode(SENSOR_PIN_3, INPUT);
    pinMode(SENSOR_PIN_4, INPUT);
    pinMode(SENSOR_PIN_5, INPUT);
    pinMode(SENSOR_PIN_6, INPUT);
    pinMode(SENSOR_PIN_7, INPUT);
    pinMode(SENSOR_PIN_8, INPUT);
    
    pinMode(ONBOARD_LED, OUTPUT);
    
    Serial.println("✓ Sensors configured (Digital Mode - Inverted)");
    Serial.println("  0 = White Line | 1 = Black Surface\n");
}

void Sensors::printCalibration() {
    Serial.println("Digital mode - No calibration needed");
}

// Read all 8 sensors as digital
// INVERTED: 0 = white line, 1 = black surface
void Sensors::readRaw(uint16_t* values) {
    values[0] = digitalRead(SENSOR_PIN_1);  // Right outer
    values[1] = digitalRead(SENSOR_PIN_2);
    values[2] = digitalRead(SENSOR_PIN_3);
    values[3] = digitalRead(SENSOR_PIN_4);  // Center right
    values[4] = digitalRead(SENSOR_PIN_5);  // Center left
    values[5] = digitalRead(SENSOR_PIN_6);
    values[6] = digitalRead(SENSOR_PIN_7);
    values[7] = digitalRead(SENSOR_PIN_8);  // Left outer
}

void Sensors::readDigital(bool* values) {
    // INVERTED: !  to flip logic
    // We want: true = white line detected
    values[0] = digitalRead(SENSOR_PIN_1);  // Invert! 
    values[1] = digitalRead(SENSOR_PIN_2);
    values[2] = digitalRead(SENSOR_PIN_3);
    values[3] = digitalRead(SENSOR_PIN_4);
    values[4] = digitalRead(SENSOR_PIN_5);
    values[5] = digitalRead(SENSOR_PIN_6);
    values[6] = digitalRead(SENSOR_PIN_7);
    values[7] = digitalRead(SENSOR_PIN_8);
}

// ========== SIMPLE DIGITAL POSITION CALCULATION ==========

float Sensors::getPosition() {
    bool sensors[8];
    readDigital(sensors);  // true = white line
    
    // Weights for 8 sensors: -7 -5 -3 -1 +1 +3 +5 +7
    // Negative = right side, Positive = left side
    
    int weightedSum = 0;
    int activeCount = 0;
    
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {  // If sensor detects white line (inverted to true)
            weightedSum += weights[i];
            activeCount++;
        }
    }
    
    // Calculate position
    if (activeCount > 0) {
        lastPosition = (float)weightedSum / (float)activeCount;
    }
    // else: keep last known position
    
    return lastPosition;
}

float Sensors::getLineError() {
    // Error is just the position (0 = centered)
    return getPosition();
}

// ========== HELPER FUNCTIONS ==========

bool Sensors::isLineDetected(uint16_t value, uint8_t sensorIndex) {
    return (value == 0);  // INVERTED: 0 = white line detected
}

bool Sensors::onLine() {
    bool sensors[8];
    readDigital(sensors);
    
    // Check if ANY sensor sees the line
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {
            return true;
        }
    }
    return false;
}

int Sensors::getActiveSensorCount() {
    bool sensors[8];
    readDigital(sensors);
    
    int count = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {
            count++;
        }
    }
    return count;
}

// ========== JUNCTION DETECTION ==========

PathOptions Sensors::getAvailablePaths() {
    bool sensors[8];
    readDigital(sensors);
    
    PathOptions paths;
    
    // Count active sensors (detecting white line)
    int activeCount = 0;
    for (int i = 0; i < 8; i++) {
        if (sensors[i]) activeCount++;
    }
    
    // Junction = 5 or more sensors active
    // Normal line = 2-3 sensors (as you described)
    if (activeCount >= 5) {
        // LEFT: S7 or S8 active
        paths.left = (sensors[6] || sensors[7]);
        
        // RIGHT: S1 or S2 active
        paths. right = (sensors[0] || sensors[1]);
        
        // STRAIGHT: S4 or S5 active (center)
        paths.straight = (sensors[3] || sensors[4]);
    }
    else {
        // Normal line - not a junction (2-3 sensors)
        paths. left = false;
        paths. right = false;
        paths. straight = (activeCount > 0);
    }
    
    return paths;
}

JunctionType Sensors::classifyJunction(PathOptions paths) {
    int pathCount = 0;
    if (paths.left) pathCount++;
    if (paths.straight) pathCount++;
    if (paths.right) pathCount++;
    
    if (pathCount == 0) {
        return JUNCTION_DEAD_END;
    } 
    else if (pathCount == 1) {
        if (paths.left && ! paths.straight) return JUNCTION_90_LEFT;
        if (paths.right && !paths.straight) return JUNCTION_90_RIGHT;
        return JUNCTION_NONE;
    } 
    else if (pathCount == 2) {
        if (paths.left && paths.straight) return JUNCTION_T_LEFT;
        if (paths.right && paths.straight) return JUNCTION_T_RIGHT;
        if (paths.left && paths.right) return JUNCTION_T_BOTH;
        return JUNCTION_NONE;
    } 
    else if (pathCount == 3) {
        return JUNCTION_CROSS;
    }
    
    return JUNCTION_NONE;
}

// ========== FINISH DETECTION ==========

bool Sensors::isLineEnd() {
    bool sensors[8];
    readDigital(sensors);
    
    // Line end = ALL sensors see black (none active)
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {
            return false;  // Still seeing white line
        }
    }
    return true;  // All sensors see black = line end
}

bool Sensors::isEndPoint() {
    bool sensors[8];
    readDigital(sensors);
    
    // Finish square = most/all sensors see white
    int whiteCount = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (sensors[i]) {
            whiteCount++;
        }
    }
    
    // Finish square: 6-8 sensors active
    // Normal line: 2-3 sensors active
    return (whiteCount >= 6);
}

// ========== UTILITY ==========

void Sensors::getSensorArray(bool* arr) {
    readDigital(arr);
}

void Sensors::getAnalogArray(uint16_t* arr) {
    readRaw(arr);  // Returns actual digital values (0 or 1)
}

void Sensors::calibrate() {
    Serial.println("Digital mode - no calibration needed");
}