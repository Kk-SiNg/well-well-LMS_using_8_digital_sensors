/*
 * Pins.h
 * Hardware pin definitions - DIGITAL MODE
 */

#pragma once
#include <cstdint>

// === 8-CHANNEL SENSOR ARRAY (ALL DIGITAL) ===
#define SENSOR_PIN_1  25  // Rightmost
#define SENSOR_PIN_2  39
#define SENSOR_PIN_3  34
#define SENSOR_PIN_4  35  // Center-Right
#define SENSOR_PIN_5  32  // Center-Left
#define SENSOR_PIN_6  33
#define SENSOR_PIN_7  36
#define SENSOR_PIN_8  26  // Leftmost

const uint8_t SensorCount = 8;

// All sensors are digital in this mode
const bool isAnalogPin[SensorCount] = {
    false, false, false, false,
    false, false, false, false
};

// Digital mode settings
#define LINE_THRESHOLD 1              // Digital: 1 = line detected
#define MIN_DETECTION_RATIO 0.5
#define CALIBRATION_TIME_MS 1000      // Shorter since no calibration needed

// === MOTOR CONTROL (L298N) ===
#define MOTOR_L_IN1 15
#define MOTOR_L_IN2 4
#define MOTOR_L_ENA 16

#define MOTOR_R_IN3 17
#define MOTOR_R_IN4 18
#define MOTOR_R_ENB 19

// === ENCODERS ===
#define ENCODER_R_A 22
#define ENCODER_R_B 21
#define ENCODER_L_A 23
#define ENCODER_L_B 5

// === USER INTERFACE ===
#define ONBOARD_LED 2
#define USER_BUTTON 0

// === MOTOR PARAMETERS ===
// Defined in Motors.cpp

// === WIFI CONFIGURATION ===
#define SSID "Redmi A2"
#define PASSWORD "kvsandkks"
#define TELNET_PORT 23