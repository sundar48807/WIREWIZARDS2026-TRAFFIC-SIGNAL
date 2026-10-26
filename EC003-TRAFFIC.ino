#include <Wire.h>

// =====================================================
// SMART FOUR-WAY TRAFFIC LIGHT
// Arduino Uno R3 + 12 direct LEDs + 4 IR sensors
// + 1 HC-SR04 + 4 ambulance buttons + PCF8574
// Serial Monitor: 9600 baud
// =====================================================

// Direction index: 0 North, 1 South, 2 East, 3 West
const char* ROAD[4] = {"NORTH", "SOUTH", "EAST", "WEST"};

// Direct LEDs
const byte RED[4]    = {2, 5, 8, 11};
const byte YELLOW[4] = {3, 6, 9, 12};
const byte GREEN[4]  = {4, 7, 10, 13};

// Ultrasonic sensor monitors the North approach
const byte TRIG_PIN = A0;
const byte ECHO_PIN = A1;

// PCF8574 I2C expander
const byte PCF_ADDR = 0x20;

// P0-P3: IR sensors; P4-P7: ambulance buttons
const byte IR_MASK[4]  = {0x01, 0x02, 0x04, 0x08};
const byte AMB_MASK[4] = {0x10, 0x20, 0x40, 0x80};

// Change to false if your IR module outputs HIGH on detection.
const bool IR_ACTIVE_LOW = true;

// Timing settings
const unsigned long MIN_GREEN_MS = 5000UL;
const unsigned long MAX_GREEN_MS = 15000UL;
const unsigned long YELLOW_MS = 2000UL;
const unsigned long ALL_RED_MS = 2000UL;
const unsigned long AMBULANCE_GREEN_MS = 10000UL;

// HC-SR04 vehicle detection threshold
const unsigned int VEHICLE_DISTANCE_CM = 150;

// Counts waiting vehicles since the road was last served
unsigned long waitingCount[4] = {0, 0, 0, 0};
unsigned long totalCount[4] = {0, 0, 0, 0};

bool previousIR[4] = {false, false, false, false};
bool previousButton[4] = {false, false, false, false};

int currentDirection = 0;
int pendingAmbulance = -1;

enum SignalState { RED_STATE, YELLOW_STATE, GREEN_STATE };

SignalState signalState[4] = {
  RED_STATE, RED_STATE, RED_STATE, RED_STATE
};

unsigned int lastDistanceCm = 0;
bool ultrasonicVehiclePresent = false;
unsigned long lastUltrasonicRead = 0;
unsigned long ultrasonicDetectionCount = 0;

// -----------------------------------------------------
// PCF8574 INPUT READING
// -----------------------------------------------------
byte readPCF() {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(0xFF); // Release pins for input
  if (Wire.endTransmission() != 0) {
    return 0xFF;
  }

  Wire.requestFrom(PCF_ADDR, (byte)1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

// -----------------------------------------------------
// TRAFFIC LIGHT CONTROL
// -----------------------------------------------------
void setAllRed() {
  for (byte i = 0; i < 4; i++) {
    digitalWrite(GREEN[i], LOW);
    digitalWrite(YELLOW[i], LOW);
    digitalWrite(RED[i], HIGH);
    signalState[i] = RED_STATE;
  }
}

void setGreen(byte direction) {
  setAllRed();

  digitalWrite(RED[direction], LOW);
  digitalWrite(GREEN[direction], HIGH);
  signalState[direction] = GREEN_STATE;
}

void setYellow(byte direction) {
  setAllRed();

  digitalWrite(RED[direction], LOW);
  digitalWrite(YELLOW[direction], HIGH);
  signalState[direction] = YELLOW_STATE;
}

const char* stateName(byte direction) {
  switch (signalState[direction]) {
    case GREEN_STATE:  return "GREEN - GO";
    case YELLOW_STATE: return "YELLOW - WAIT";
    default:           return "RED - STOP";
  }
}

// -----------------------------------------------------
// ULTRASONIC VEHICLE DETECTION
// -----------------------------------------------------
unsigned int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 25000UL);

  if (duration == 0) {
    return 0; // No echo
  }

  // HC-SR04 distance conversion: microseconds / 58
  return duration / 58UL;
}

void updateUltrasonic() {
  // Avoid measuring too frequently.
  if (millis() - lastUltrasonicRead < 100UL) {
    return;
  }

  lastUltrasonicRead = millis();
  lastDistanceCm = readDistanceCm();

  bool detected =
    lastDistanceCm >= 2 &&
    lastDistanceCm <= VEHICLE_DISTANCE_CM;

  // Print only when the detection state changes.
  if (detected && !ultrasonicVehiclePresent) {
    ultrasonicDetectionCount++;

    Serial.print(F("[ULTRASONIC DETECTED] NORTH | Distance: "));
    Serial.print(lastDistanceCm);
    Serial.println(F(" cm"));
  }

  if (!detected && ultrasonicVehiclePresent) {
    Serial.println(F("[ULTRASONIC CLEAR] NORTH approach"));
  }

  ultrasonicVehiclePresent = detected;
}

// -----------------------------------------------------
// IR VEHICLE COUNTING AND AMBULANCE BUTTONS
// -----------------------------------------------------
void checkSensors() {
  byte inputs = readPCF();

  for (byte i = 0; i < 4; i++) {
    bool rawIR = !(inputs & IR_MASK[i]);
    bool detectedIR = IR_ACTIVE_LOW ? rawIR : !rawIR;

    // Count once on the clear-to-detected transition.
    if (detectedIR && !previousIR[i]) {
      waitingCount[i]++;
      totalCount[i]++;

      Serial.print(F("[IR VEHICLE DETECTED] "));
      Serial.print(ROAD[i]);
      Serial.print(F(" | Waiting: "));
      Serial.print(waitingCount[i]);
      Serial.print(F(" | Total: "));
      Serial.println(totalCount[i]);

      // Show whether the North ultrasonic sensor also sees
      // a nearby object. It does not count the same vehicle twice.
      if (i == 0) {
        if (ultrasonicVehiclePresent) {
          Serial.println(
            F("  -> Ultrasonic confirmation: NEARBY OBJECT")
          );
        } else {
          Serial.println(
            F("  -> Ultrasonic confirmation: NOT CURRENTLY DETECTED")
          );
        }
      }
    }

    previousIR[i] = detectedIR;

    bool buttonPressed = !(inputs & AMB_MASK[i]);

    if (buttonPressed && !previousButton[i]) {
      if (pendingAmbulance == -1) {
        pendingAmbulance = i;

        Serial.println();
        Serial.println(F("***** AMBULANCE REQUEST *****"));
        Serial.print(F("Priority requested: "));
        Serial.println(ROAD[i]);
        Serial.println(F("Will be served after a safe transition."));
      } else {
        Serial.print(F("Request already queued for "));
        Serial.println(ROAD[pendingAmbulance]);
      }
    }

    previousButton[i] = buttonPressed;
  }
}

// -----------------------------------------------------
// SERIAL MONITOR DASHBOARD
// -----------------------------------------------------
void printDashboard(const char* event, unsigned long greenMs) {
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("          SMART TRAFFIC CONTROL SYSTEM"));
  Serial.println(F("================================================"));

  Serial.print(F("Current event: "));
  Serial.println(event);

  Serial.println(F("---------------- SIGNAL STATUS ----------------"));

  for (byte i = 0; i < 4; i++) {
    Serial.print(ROAD[i]);
    Serial.print(F(" : "));
    Serial.print(stateName(i));
    Serial.print(F(" | Waiting vehicles: "));
    Serial.println(waitingCount[i]);
  }

  Serial.println(F("---------------- SENSOR STATUS ----------------"));

  Serial.print(F("Ultrasonic - NORTH: "));
  if (lastDistanceCm == 0) {
    Serial.println(F("No echo"));
  } else {
    Serial.print(lastDistanceCm);
    Serial.println(F(" cm"));
  }

  Serial.print(F("Ultrasonic nearby object: "));
  Serial.println(
    ultrasonicVehiclePresent ? F("YES") : F("NO")
  );

  Serial.print(F("Ultrasonic detection events: "));
  Serial.println(ultrasonicDetectionCount);

  Serial.println(F("---------------- TIMING STATUS ----------------"));

  Serial.print(F("Green duration: "));
  Serial.print(greenMs / 1000UL);
  Serial.println(F(" seconds"));

  Serial.print(F("Ambulance request: "));
  if (pendingAmbulance == -1) {
    Serial.println(F("NONE"));
  } else {
    Serial.println(ROAD[pendingAmbulance]);
  }

  Serial.println(F("================================================"));
}

// -----------------------------------------------------
// DELAY WHILE CONTINUING TO CHECK SENSORS
// -----------------------------------------------------
void serviceDelay(unsigned long duration) {
  unsigned long start = millis();

  while (millis() - start < duration) {
    checkSensors();
    updateUltrasonic();
    delay(10);
  }
}

// -----------------------------------------------------
// ADAPTIVE GREEN TIMING
// -----------------------------------------------------
unsigned long calculateGreenTime(byte direction) {
  unsigned long seconds = 5UL + waitingCount[direction];

  if (seconds < 5UL) {
    seconds = 5UL;
  }

  if (seconds > 15UL) {
    seconds = 15UL;
  }

  return seconds * 1000UL;
}

// -----------------------------------------------------
// YELLOW AND ALL-RED TRANSITION
// -----------------------------------------------------
void safeTransition(byte direction) {
  setYellow(direction);
  printDashboard("YELLOW - PREPARE TO STOP", 0);
  serviceDelay(YELLOW_MS);

  setAllRed();
  printDashboard("ALL RED - SAFETY INTERVAL", 0);
  serviceDelay(ALL_RED_MS);
}

// -----------------------------------------------------
// NORMAL TRAFFIC PHASE
// -----------------------------------------------------
void runNormalPhase(byte direction) {
  unsigned long greenMs = calculateGreenTime(direction);

  Serial.println();
  Serial.print(F("[NORMAL TRAFFIC] "));
  Serial.println(ROAD[direction]);

  setGreen(direction);
  printDashboard("NORMAL GREEN", greenMs);
  serviceDelay(greenMs);

  // End this road's green phase safely.
  safeTransition(direction);

  // Reset the waiting queue after serving this road.
  waitingCount[direction] = 0;
}

// -----------------------------------------------------
// AMBULANCE PRIORITY PHASE
// -----------------------------------------------------
void runAmbulancePhase(byte direction) {
  Serial.println();
  Serial.println(F("**********************************************"));
  Serial.println(F("        AMBULANCE PRIORITY ACTIVATED"));
  Serial.print(F("        PRIORITY ROAD: "));
  Serial.println(ROAD[direction]);
  Serial.println(F("**********************************************"));

  // The normal phase has already completed its safe
  // yellow/all-red transition before this function runs.

  setAllRed();
  printDashboard("AMBULANCE - ALL RED SAFETY INTERVAL", 0);
  serviceDelay(ALL_RED_MS);

  setGreen(direction);
  printDashboard("AMBULANCE PRIORITY GREEN", AMBULANCE_GREEN_MS);
  serviceDelay(AMBULANCE_GREEN_MS);

  safeTransition(direction);
  waitingCount[direction] = 0;

  Serial.println(F("[AMBULANCE PRIORITY COMPLETE]"));
  Serial.println(F("Returning to normal traffic sequence."));
}

// -----------------------------------------------------
// SETUP
// -----------------------------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  for (byte i = 0; i < 4; i++) {
    pinMode(RED[i], OUTPUT);
    pinMode(YELLOW[i], OUTPUT);
    pinMode(GREEN[i], OUTPUT);
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  setAllRed();

  Serial.println();
  Serial.println(F("SMART TRAFFIC CONTROLLER STARTED"));
  Serial.println(F("Serial Monitor: 9600 baud"));
  Serial.println(F("Normal sequence: NORTH -> SOUTH -> EAST -> WEST"));
  Serial.println(F("Green: 5-15 seconds, based on IR vehicle count"));
  Serial.println(F("Yellow: 2 seconds"));
  Serial.println(F("All red: 2 seconds"));
  Serial.println(F("Ultrasonic sensor monitors NORTH only."));
  Serial.println(F("All signals initialized to RED."));
}

// -----------------------------------------------------
// MAIN LOOP
// -----------------------------------------------------
void loop() {
  checkSensors();
  updateUltrasonic();

  if (pendingAmbulance >= 0) {
    byte requestedDirection = pendingAmbulance;
    pendingAmbulance = -1;

    runAmbulancePhase(requestedDirection);

    currentDirection = (requestedDirection + 1) % 4;
    return;
  }

  runNormalPhase(currentDirection);
  currentDirection = (currentDirection + 1) % 4;
}