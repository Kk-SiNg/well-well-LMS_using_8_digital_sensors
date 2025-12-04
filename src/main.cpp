#include <Arduino.h>
/*
 * IIT Bombay Mesmerize Line Follower
 * Fresh implementation with digital sensors
 * Simple PID + LSRB junction navigation
 */

#include <WiFi.h>
#include "Pins.h"
#include "Sensors.h"
#include "Motors.h"

// === WiFi Server ===
WiFiServer server(TELNET_PORT);
WiFiClient client;

// === Global Objects ===
Sensors sensors;
Motors motors;

// === Simple PID Variables ===
float Kp = 30.0;   // Proportional gain
float Ki = 0.0;    // Integral gain (start with 0)
float Kd = 15.0;   // Derivative gain
float lastError = 0;
float integral = 0;
float maxIntegral = 1000;  // Prevent integral windup

int baseSpeed = 150;
int turnSpeed = 190;
int maxSpeed = 220;

// === Junction Settings ===
unsigned long junctionDebounce = 500;  // ms between junction detections
unsigned long lastJunctionTime = 0;
int junctionCount = 0;

// === Finish Detection ===
unsigned long finishDetectTime = 0;
const unsigned long FINISH_CONFIRM_MS = 300;  // Confirm finish for 300ms

// === Robot State ===
enum State {
    IDLE,
    RUNNING,
    FINISHED
};
State currentState = IDLE;
bool robotRunning = false;

// === Timing ===
unsigned long runStartTime = 0;
unsigned long lastWiFiUpdate = 0;
unsigned long lastDebugPrint = 0;

// === Emergency Stop ===
unsigned long buttonPressStart = 0;

// === Function Declarations ===
void setupWiFi();
void handleWiFiClient();
void processCommand(String cmd);
void printMenu();
void printStatus();
void runPID();
String junctionTypeToString(JunctionType type);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  IIT Bombay Mesmerize Line Follower  ║");
    Serial.println("║  Digital Sensors + Simple PID         ║");
    Serial.println("╚════════════════════════════════════════╝\n");
    
    pinMode(ONBOARD_LED, OUTPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    
    // Initialize motors and sensors
    motors.setup();
    sensors.setup();
    
    setupWiFi();
    
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  Configuration                         ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║  PID: Kp=%.1f Ki=%.1f Kd=%.1f          ║\n", Kp, Ki, Kd);
    Serial.printf("║  Base Speed: %d                        ║\n", baseSpeed);
    Serial.printf("║  Junction Debounce: %lums              ║\n", junctionDebounce);
    Serial.println("╚════════════════════════════════════════╝\n");
    
    currentState = IDLE;
    Serial.println("✓ Ready!  Press button or send START command\n");
}

void loop() {
    yield();  // Let WiFi handle its tasks
    handleWiFiClient();
    
    // === WiFi Telemetry (every 500ms) ===
    if (millis() - lastWiFiUpdate > 500 && client && client.connected()) {
        bool sensorVals[8];
        sensors. getSensorArray(sensorVals);
        
        client.print("S:[");
        for (int i = 0; i < 8; i++) {
            client.print(sensorVals[i] ? "█" : "·");
        }
        client. printf("] Err:%.1f Spd:%d Junc:%d | ", 
                     sensors.getLineError(), baseSpeed, junctionCount);
        
        switch(currentState) {
            case IDLE: client.print("IDLE"); break;
            case RUNNING: client. print("RUNNING"); break;
            case FINISHED: client.print("FINISHED"); break;
        }
        client.println();
        lastWiFiUpdate = millis();
    }
    
    // === Emergency Stop (2 second button press) ===
    if (digitalRead(USER_BUTTON) == LOW) {
        if (buttonPressStart == 0) {
            buttonPressStart = millis();
        } else if (millis() - buttonPressStart > 2000) {
            motors.stopBrake();
            robotRunning = false;
            currentState = FINISHED;
            Serial.println("\n⚠️ EMERGENCY STOP!");
            if (client) client.println("⚠️ EMERGENCY STOP!");
            buttonPressStart = 0;
        }
    } else {
        buttonPressStart = 0;
    }
    
    // === State Machine ===
    switch (currentState) {
        
        case IDLE:
        {
            // Wait for button press or START command
            if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                delay(50);  // Debounce
                if (digitalRead(USER_BUTTON) == LOW || robotRunning) {
                    Serial.println("\n>>> MAZE RUN STARTED!  <<<");
                    if (client) client.println("\n>>> MAZE RUN STARTED!");
                    
                    currentState = RUNNING;
                    robotRunning = true;
                    junctionCount = 0;
                    lastJunctionTime = 0;
                    finishDetectTime = 0;
                    runStartTime = millis();
                    
                    // Reset PID
                    lastError = 0;
                    integral = 0;
                    motors.clearEncoders();
                    
                    // Wait for button release
                    while(digitalRead(USER_BUTTON) == LOW) delay(10);
                }
            }
            break;
        }
            
        case RUNNING:
        {
            if (! robotRunning) {
                motors.stopBrake();
                currentState = FINISHED;
                Serial.println("\n>>> STOPPED");
                if (client) client. println("\n>>> STOPPED");
                break;
            }
            
            // === Check for FINISH WHITE SQUARE ===
            if (sensors.isEndPoint()) {
                if (finishDetectTime == 0) {
                    finishDetectTime = millis();
                    Serial.println("⚠️ Finish square detected.. .");
                } 
                else if (millis() - finishDetectTime > FINISH_CONFIRM_MS) {
                    // CONFIRMED FINISH! 
                    motors.stopBrake();
                    robotRunning = false;
                    
                    unsigned long runTime = (millis() - runStartTime) / 1000;
                    
                    Serial.println("\n╔════════════════════════════════════════╗");
                    Serial.println("║      🏆  MAZE COMPLETE!  🏆            ║");
                    Serial. println("║   IIT Bombay Mesmerize Complete!       ║");
                    Serial. println("╚════════════════════════════════════════╝");
                    Serial.printf("\n⏱️  Time: %lu seconds\n", runTime);
                    Serial.printf("🔀 Junctions: %d\n\n", junctionCount);
                    
                    if (client) {
                        client.println("\n🏆 MAZE COMPLETE!");
                        client.printf("Time: %lus | Junctions: %d\n", runTime, junctionCount);
                    }
                    
                    currentState = FINISHED;
                    finishDetectTime = 0;
                    break;
                }
            } else {
                finishDetectTime = 0;  // Reset if not on finish square
            }
            
            // === Run PID Line Following ===
            runPID();
            
            // === Junction Detection ===
            if (millis() - lastJunctionTime > junctionDebounce) {
                PathOptions paths = sensors.getAvailablePaths();
                
                // Count available paths
                int pathCount = 0;
                if (paths.left) pathCount++;
                if (paths.right) pathCount++;
                if (paths.straight) pathCount++;
                
                // Is this a junction?  (more than just straight OR only left/right)
                bool isJunction = (pathCount > 1) || (pathCount == 1 && ! paths.straight);
                
                if (isJunction) {
                    motors.stopBrake();
                    delay(100);
                    
                    junctionCount++;
                    
                    JunctionType jType = sensors.classifyJunction(paths);
                    
                    if (client) {
                        client.printf("J%d: ", junctionCount);
                        if(paths.left) client.print("L");
                        if(paths.straight) client.print("S");
                        if(paths. right) client.print("R");
                        client.println();
                    }
                    
                    // Move to junction center
                    motors.moveForward(TICKS_TO_CENTER);
                    delay(1000);
                    bool sensorVals[8];
                    sensors.getSensorArray(sensorVals);
                    
                    // === LSRB Logic: Left > Straight > Right > Back ===
                    if (paths.left) {
                        motors.turn_90_left();
                    }
                    else if (sensorVals[3] && sensorVals[4]) {
                        // No turn needed
                    }
                    else if (paths.right) {
                        motors. turn_90_right();
                    }
                    else {
                        motors.turn_180_back();
                    }
                    
                    // Reset after junction
                    motors.clearEncoders();
                    lastError = 0;
                    integral = 0;
                    lastJunctionTime = millis();
                    delay(100);
                }
            }
            
            break;
        }
            
        case FINISHED:
        {
            // Victory blink
            digitalWrite(ONBOARD_LED, ! digitalRead(ONBOARD_LED));
            delay(200);
            break;
        }
    }
}

// === PID Line Following ===
void runPID() {
    // Get current error from sensors
    float error = sensors. getLineError();
    
    // PID calculations
    float P = Kp * error;
    
    integral += error;
    // Anti-windup: constrain integral
    integral = constrain(integral, -maxIntegral, maxIntegral);
    float I = Ki * integral;
    
    float D = Kd * (error - lastError);
    
    // Total correction
    float correction = P + I + D;
    
    // Constrain correction
    correction = constrain(correction, -baseSpeed, baseSpeed);
    
    // Apply to motors
    int leftSpeed = baseSpeed - (int)correction;
    int rightSpeed = baseSpeed + (int)correction;
    
    // Constrain motor speeds
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
    
    motors.setSpeeds(leftSpeed, rightSpeed);
    
    // Save for next iteration
    lastError = error;
}

String junctionTypeToString(JunctionType type) {
    switch(type) {
        case JUNCTION_T_LEFT: return "T-Left ├";
        case JUNCTION_T_RIGHT: return "T-Right ┤";
        case JUNCTION_T_BOTH: return "T-Both ┬";
        case JUNCTION_CROSS: return "Cross ┼";
        case JUNCTION_90_LEFT: return "90° Left └";
        case JUNCTION_90_RIGHT: return "90° Right ┘";
        case JUNCTION_DEAD_END: return "Dead End";
        default: return "Straight";
    }
}

void setupWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✓ WiFi Connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Connect: telnet ");
        Serial.println(WiFi.localIP());
        server.begin();
    } else {
        Serial.println("❌ WiFi Failed");
    }
}

void handleWiFiClient() {
    if (! client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("WiFi client connected");
            client.println("╔════════════════════════════════════════╗");
            client.println("║  Mesmerize Line Follower Console     ║");
            client. println("╚════════════════════════════════════════╝");
            printMenu();
        }
    }
    
    if (client && client.connected() && client.available()) {
        String cmd = client.readStringUntil('\n');
        cmd.trim();
        processCommand(cmd);
    }
}

void processCommand(String cmd) {
    cmd.toUpperCase();
    
    if (cmd == "START" || cmd == "GO") {
        if (currentState == IDLE || currentState == FINISHED) {
            robotRunning = true;
            client.println("✓ STARTING.. .");
        } else {
            client.println("❌ Already running");
        }
    }
    else if (cmd == "STOP" || cmd == "S") {
        robotRunning = false;
        motors.stopBrake();
        currentState = FINISHED;
        client.println("✓ STOPPED");
    }
    else if (cmd == "RESET" || cmd == "R") {
        currentState = IDLE;
        robotRunning = false;
        motors.stopBrake();
        lastError = 0;
        integral = 0;
        junctionCount = 0;
        client.println("✓ RESET - Ready to start");
    }
    else if (cmd == "STATUS" || cmd == "ST") {
        printStatus();
    }
    else if (cmd == "HELP" || cmd == "H") {
        printMenu();
    }
    
    // === PID TUNING ===
    else if (cmd. startsWith("KP ")) {
        Kp = cmd.substring(3).toFloat();
        client.printf("✓ Kp = %.2f\n", Kp);
        Serial.printf("WiFi: Kp = %.2f\n", Kp);
    }
    else if (cmd.startsWith("KI ")) {
        Ki = cmd. substring(3).toFloat();
        client.printf("✓ Ki = %.3f\n", Ki);
        Serial.printf("WiFi: Ki = %.3f\n", Ki);
    }
    else if (cmd.startsWith("KD ")) {
        Kd = cmd.substring(3).toFloat();
        client.printf("✓ Kd = %. 2f\n", Kd);
        Serial.printf("WiFi: Kd = %.2f\n", Kd);
    }
    else if (cmd.startsWith("TUNE ")) {
        int s1 = cmd.indexOf(' ', 5);
        int s2 = cmd.indexOf(' ', s1 + 1);
        if (s1 > 0 && s2 > 0) {
            Kp = cmd.substring(5, s1).toFloat();
            Ki = cmd.substring(s1 + 1, s2).toFloat();
            Kd = cmd.substring(s2 + 1).toFloat();
            client.printf("✓ PID: Kp=%.1f Ki=%.2f Kd=%.1f\n", Kp, Ki, Kd);
            integral = 0;  // Reset integral when tuning
        }
    }
    
    // === SPEED TUNING ===
    else if (cmd.startsWith("SPEED ")) {
        baseSpeed = cmd.substring(6). toInt();
        baseSpeed = constrain(baseSpeed, 50, maxSpeed);
        client.printf("✓ Speed = %d\n", baseSpeed);
    }
    else if (cmd. startsWith("DEBOUNCE ")) {
        junctionDebounce = cmd.substring(9).toInt();
        junctionDebounce = constrain(junctionDebounce, 100, 1000);
        client.printf("✓ Debounce = %lums\n", junctionDebounce);
    }
    
    // === TESTING ===
    else if (cmd == "TEST" || cmd == "T") {
        bool sensors_arr[8];
        sensors.getSensorArray(sensors_arr);
        
        client.println("\n=== Sensor Test ===");
        client.print("Pattern: [");
        for (int i = 0; i < 8; i++) {
            client.print(sensors_arr[i] ?  "█" : "·");
        }
        client.println("]");
        client.printf("Error: %.2f\n", sensors.getLineError());
        client.printf("Position: %. 2f\n", sensors.getPosition());
        client.printf("Active: %d sensors\n", sensors.getActiveSensorCount());
        client.printf("On Line: %s\n", sensors.onLine() ? "YES" : "NO");
        client.printf("Finish: %s\n", sensors. isEndPoint() ? "YES" : "NO");
        client.println("==================\n");
    }
    else if (cmd == "PID") {
        client.println("\n=== PID Values ===");
        client.printf("Kp = %.2f\n", Kp);
        client.printf("Ki = %.3f\n", Ki);
        client.printf("Kd = %.2f\n", Kd);
        client.printf("Last Error = %.2f\n", lastError);
        client.printf("Integral = %.2f\n", integral);
        client. println("==================\n");
    }
    else {
        client.println("❌ Unknown.  Type HELP");
    }
}

void printMenu() {
    client.println("\n=== Commands ===");
    client.println("START / STOP / RESET");
    client.println("STATUS - Show status");
    client.println("TEST - Test sensors");
    client.println("PID - Show PID values");
    client.println("");
    client.println("=== Tuning ===");
    client.println("KP <val> / KI <val> / KD <val>");
    client.println("TUNE <kp> <ki> <kd>");
    client.println("SPEED <val>");
    client.println("DEBOUNCE <ms>");
    client.println("================\n");
}

void printStatus() {
    client.println("\n=== Status ===");
    client.print("State: ");
    switch(currentState) {
        case IDLE: client.println("IDLE"); break;
        case RUNNING: client.println("RUNNING"); break;
        case FINISHED: client.println("FINISHED"); break;
    }
    client.printf("PID: Kp=%.1f Ki=%.2f Kd=%.1f\n", Kp, Ki, Kd);
    client.printf("Speed: %d\n", baseSpeed);
    client.printf("Debounce: %lums\n", junctionDebounce);
    client.printf("Error: %.2f\n", sensors.getLineError());
    client.printf("Junctions: %d\n", junctionCount);
    client.printf("On Line: %s\n", sensors.onLine() ? "YES" : "NO");
    client. println("==============\n");
}