/*
 * ======================================================================================
 *                              FILE NAME: MotorDriver.ino
 * ======================================================================================
 *
 * BEGINNER DESCRIPTION:
 * This is a program for an Arduino Mega. The Arduino is like the electronic "brain"
 * that controls the robotic arm.
 *
 * What this program does:
 * 1. Receives orders from the "vision brain" (the Raspberry Pi or computer).
 * 2. Orders come in the form of a text message (ex: "$1,0,1,1,1,1,1,90").
 * 3. The Arduino reads this message and translates it into physical movements in the motors.
 *
 * TYPES OF MOTORES USED:
 * - Servo Motors (controlled by PCA9685 board): Used for fingers, elbow and hand rotation.
 * - Stepper Motor (controlled by another Arduino Uno): Used to rotate the base of the arm.
 *
 * LANGUAGE: C++ (Standard Arduino language)
 * ======================================================================================
 */

// --- LIBRARIES (Code Collections) ---
// These lines "call" extra code already written by others
// to help us talk to specific components.
#include <Wire.h>                     // Allows I2C communication (used to talk to servo board)
#include <Adafruit_PWMServoDriver.h>  // Specific library to control servo board (PCA9685)

// Create the "object" that controls the servos. Imagine this as hiring a manager for motors.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- CALIBRATION (Mechanical Adjustments) ---
// These values define physical limits of motors so they don't break the arm.
// A servo motor moves based on a "pulse" signal.
#define PULSE_MIN  100  // Pulse value for motor to go to MINIMUM position
#define PULSE_MAX  500  // Pulse value for motor to go to MAXIMUM position

// --- GLOBAL VARIABLES ---
// Drawers in memory to store numbers we will use often.
int PULSE_OPEN;   // Will store the value needed to OPEN the hand (90 degrees)
int PULSE_CLOSED; // Will store the value needed to CLOSE the hand (0 degrees)

// --- PIN DEFINITION (Where we plug cables) ---
// PCA9685 board has ports numbered 0 to 15.
// Here we give names to those numbers to make code easier to read.
#define PIN_ELBOW   0   // Elbow motor connected to port 0
#define PIN_THUMB   13  // Thumb
#define PIN_INDEX   11  // Index Finger
#define PIN_MIDDLE  10  // Middle Finger
#define PIN_RING    15  // Ring Finger
#define PIN_PINKY   14  // Pinky Finger
#define PIN_ROTATION 12 // Motor that rotates the wrist

// --- COMMUNICATION (Computer Talk) ---
// Variables to receive text from Raspberry Pi
char g_pcBuffer[100];          // A "sentence" can have up to 100 letters
bool g_messageReady = false;   // "Flag" that goes up when a complete sentence arrived
int g_bufferIndex = 0;         // Counter to know which letter we are at

/*
 * ======================================================================================
 * SETUP FUNCTION (Initial Configuration)
 * This function runs only ONCE when we turn on the Arduino.
 * Used to prepare everything before starting work.
 * ======================================================================================
 */
void setup() {
  // 1. Start Communications (Radio Channels)
  // Serial:  Connection to computer (USB cable) to see error messages on screen (Debug).
  // Serial2: Connection to Raspberry Pi (the vision brain).
  // Serial3: Connection to Arduino Uno (which controls rotating base).
  
  Serial.begin(115200);   // Fast speed for PC
  Serial2.begin(115200);  // Fast speed for Raspberry Pi
  Serial3.begin(9600);    // Normal speed for Arduino Uno

  // 2. Start Servo Controller (PCA9685)
  pwm.begin();           // Wakes up servo board
  pwm.setPWMFreq(50);    // Sets frequency to 50Hz (standard for analog servos)
  Wire.setClock(400000); // Speeds up I2C communication to respond faster

  // 3. POSITION PRE-CALCULATION
  // The 'map' function translates degrees (0-180) to motor pulse values (100-500).
  // We calculate this now so we don't have to do math all the time.
  PULSE_OPEN   = map(0, 0, 180, PULSE_MIN, PULSE_MAX);   // 0 degrees = Open
  PULSE_CLOSED = map(180, 0, 180, PULSE_MIN, PULSE_MAX); // 180 degrees = Closed

  delay(10); // Small pause to ensure everything stabilizes

  // 4. Start Position
  moveAll(0); // Sends all servos to position 0 (rest)
  
  // Messages for human to read on PC screen
  Serial.println("MEGA READY (TURBO MODE 115200) + ROTATION + DEBUG ACTIVATED");
  Serial.println("Example debug messages: ");
  Serial.println("$1,0,1,1,1,1,1,90");
  Serial.println("Waiting for commands...");
}

/*
 * ======================================================================================
 * LOOP FUNCTION (Infinite Cycle)
 * This function repeats forever, thousands of times per second.
 * This is where the Arduino "lives" and works.
 * ======================================================================================
 */
void loop() {
  // --- PART 1: LISTEN TO RASPBERRY PI ---
  // While there is data coming from Pi and message is not finished...
  while (Serial2.available() > 0 && !g_messageReady) {
    char inChar = Serial2.read(); // Reads one letter
    
    if (inChar == '$') {
      // The symbol '$' marks the START of a new message.
      // If we see this, restart counter to write from zero.
      g_bufferIndex = 0;
    } 
    else if (inChar == '\n') {
      // The symbol '\n' (new line) marks the END of message.
      g_pcBuffer[g_bufferIndex] = '\0'; // Closes the "string" (text) correctly
      g_messageReady = true;            // Raises flag: "We have an order!"
    } 
    else if (g_bufferIndex < 99) {
      // If not start or end, it is content (ex: '1', ',', '9').
      // Save in buffer and advance counter.
      g_pcBuffer[g_bufferIndex] = inChar;
      g_bufferIndex++;
    }
  }

  // --- PART 2: EXECUTE ORDER ---
  // If flag is raised (complete message received)...
  if (g_messageReady) {
    processMessageFast(); // Calls function that distributes tasks
    g_messageReady = false;  // Lowers flag and prepares for next
    g_bufferIndex = 0;
  }
}

/*
 * ======================================================================================
 * FUNCTION: processMessageFast
 * Objective: Take received text and command each motor individually.
 * Expected message format: "$orient,flex,d1,d2,d3,d4,d5,rotation"
 * 
 * Example: "1,0,1,1,1,1,1,90"
 * Means:
 *   - Orientation (Base): 1
 *   - Flexion (Elbow): 0
 *   - Fingers (1-5): All 1 (closed/open depending on logic)
 *   - Rotation: 90 degrees
 * ======================================================================================
 */
void processMessageFast() {
  // DEBUG: Shows on PC screen what Arduino just heard
  Serial.print("RX (Received): ");
  Serial.println(g_pcBuffer);
  
  // The 'strtok' function serves to "break" text whenever it finds a comma.
  
  // --- STEP 1: BASE (Arduino Uno) ---
  // Takes first part of text (before first comma)
  char* token = strtok(g_pcBuffer, ",");
  if (token == NULL) return; // If nothing there, cancel (safety)
  
  // Sends that order to other Arduino (Uno) that controls base
  Serial3.write(token[0]); 
  Serial.print("BASE -> Uno: ");
  Serial.println(token[0]); 
  
  // --- STEP 2: SERVOS (Hand and Elbow) ---
  // For each component, we read next piece of text and move motor.
  
  // Elbow (Digital: 0 or 1)
  moveDigital(PIN_ELBOW); 
  
  // Fingers (Digital: 0 or 1)
  moveDigital(PIN_THUMB);  // Thumb
  moveDigital(PIN_INDEX);  // Index
  moveDigital(PIN_MIDDLE); // Middle
  moveDigital(PIN_RING);   // Ring
  moveDigital(PIN_PINKY);  // Pinky
  
  // --- STEP 3: WRIST ROTATION (Analog: 0 to 180 degrees) ---
  moveAnalog(PIN_ROTATION); 
}

/*
 * ======================================================================================
 * FUNCTION: moveDigital
 * Objective: Control motors that only have two states (Open/Closed).
 * Arguments: 'porta' (which motor we will move).
 * ======================================================================================
 */
inline void moveDigital(int pin) {
  // 'strtok(NULL, ",")' asks for NEXT piece of text after last comma found
  char* token = strtok(NULL, ",");
  
  if (token != NULL) {
    // Checks if order is "1" or "0"
    // Syntax: (condition) ? value_if_true : value_if_false
    int pulse = (token[0] == '1') ? PULSE_CLOSED : PULSE_OPEN;
    
    // Sends order to servo board
    pwm.setPWM(pin, 0, pulse);
    
    // Debug message for technician to see on PC
    Serial.print("D["); Serial.print(pin); Serial.print("]: ");
    Serial.print(token[0] == '0' ? "CLOSED" : "OPEN");
    Serial.print(" ("); Serial.print(pulse); Serial.println(")");
  }
}

/*
 * ======================================================================================
 * FUNCTION: moveAnalog
 * Objective: Control motors that can be in any angle (0-180).
 * USED FOR: Wrist rotation.
 * ======================================================================================
 */
inline void moveAnalog(int pin) {
  char* token = strtok(NULL, ",");
  
  if (token != NULL) {
    // 'atoi' converts text ("123") to integer number (123)
    int angle = atoi(token); 
    
    // SAFETY: Prevent motor from trying to go beyond physical limits
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Translates angle (0-180) to motor "language" (pulses)
    int pulse = map(angle, 0, 180, PULSE_MIN, PULSE_MAX);
    
    // Executes movement
    pwm.setPWM(pin, 0, pulse);

    Serial.print("ROT["); Serial.print(pin); Serial.print("]: ");
    Serial.print(angle);
    Serial.print(" deg ("); Serial.print(pulse); Serial.println(")");
  }
}

/*
 * ======================================================================================
 * FUNCTION: moveAll
 * Objective: Move all motors to specific angle at once.
 * Useful for initialization.
 * ======================================================================================
 */
void moveAll(int ang) {
  int pulse = map(ang, 0, 180, PULSE_MIN, PULSE_MAX);
  // Loop 'for': Repeats for each motor from 0 to 6
  for (int i = 0; i <= 6; i++) { 
    pwm.setPWM(i, 0, pulse);
  }
}
