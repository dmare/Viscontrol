/**
 * Burner Control System for Arduino UNO with Atmel 328p
 *  Hi Marty
 * This program implements a state machine for controlling a burner system
 * based on various inputs, outputs, timers, and counters.
 * 
 * All inputs and outputs are simulated via the serial console.
 */
// States for the main state machine
enum states {
  STOPPED,
  START_SEQUENCE,
  STOP_SEQUENCE,
  PURGE,
  IGNITION,
  RUNNING,
  FAULT
};
// Current state of the system
enum states current_state = STOPPED;
// State names for display
const char* state_strings[] = {
  "STOPPED",
  "START_SEQUENCE",
  "STOP_SEQUENCE",
  "PURGE",
  "IGNITION",
  "RUNNING",
  "FAULT"
};
// Input states
bool MSTART = false;  // Momentary Start Switch
bool MSTOP = false;   // Momentary Stop Switch
bool PCFS = false;    // Primary Combustion Flame Sensor
bool SCFS = false;    // Secondary Combustion Flame Sensor
int LTS = 0;          // Load Temperature Sensor (value in degrees)
int FTS = 0;          // Flame Temperature Sensor (value in degrees)
// Output states
bool BLOW = false;    // Combustion Blower Power
bool SHAFT = false;   // Main Shaft Power
bool IGNITE = false;  // Hot Surface Ignitor Power
bool LOGAS = false;   // Preheat fuel gas low rate
bool HIGAS = false;   // Preheat fuel gas high rate
bool FUEL1 = false;   // Main Fuel 1
bool FUEL2 = false;   // Main Fuel 2
bool FUEL3 = false;   // Main Fuel 3
bool H2O = false;     // Cleaning Steam/Water Supply
bool IFFI = false;    // Ignition Failure Fault Indicator
bool SCFI = false;    // Secondary Combustion Fault Indicator
// Timer variables (all in milliseconds)
unsigned long PADT = 5000;    // Purge Air Delay Timer
unsigned long HSOT = 3000;    // Hot Surface Ignitor Warm Up Timer
unsigned long PHDT = 2000;    // Pre Heat Delay Timer
unsigned long MFDT = 4000;    // Main Fuel Turn On Delay Timer
unsigned long PCST = 5000;    // Primary Combustion Safety Timer
unsigned long SCST = 5000;    // Secondary Combustion Safety Timer
// Counter variables
int PCFC = 0;    // Primary Combustion Flame Sensor Count
int SCFC = 0;    // Secondary Combustion Flame Sensor Count
// Timer tracking variables
unsigned long timer_start = 0;
unsigned long current_time = 0;
bool timer_running = false;
unsigned long timer_duration = 0;
// Serial communication
char serial_buffer[150];
String serial_input = "";
bool serial_complete = false;
// ---------- HARDWARE PIN ASSIGNMENTS ----------
const uint8_t PIN_MSTART = 2;   // push-button to GND
const uint8_t PIN_MSTOP  = 3;   // push-button to GND
const uint8_t PIN_PCFS   = 4;   // toggle switch to GND
const uint8_t PIN_SCFS   = 5;   // toggle switch to GND
const uint8_t PIN_LTS    = A0;  // 10 k pot 5 V-wiper-GND
const uint8_t PIN_FTS    = A1;  // 10 k pot 5 V-wiper-GND
const uint8_t PIN_BLOW   = 8;   // LED/relay
const uint8_t PIN_SHAFT  = 9;
const uint8_t PIN_IGNITE = 10;
const uint8_t PIN_LOGAS  = 11;
const uint8_t PIN_FUEL1  = 12;
const uint8_t PIN_HIGAS  = 13;  // on-board LED
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
   // ------------ INPUTS ------------
  pinMode(PIN_MSTART, INPUT_PULLUP);
  pinMode(PIN_MSTOP , INPUT_PULLUP);
  pinMode(PIN_PCFS  , INPUT_PULLUP);
  pinMode(PIN_SCFS  , INPUT_PULLUP);
  // Analog pins (A0/A1) need no pinMode()
  // ------------ OUTPUTS -----------
  pinMode(PIN_BLOW  , OUTPUT);
  pinMode(PIN_SHAFT , OUTPUT);
  pinMode(PIN_IGNITE, OUTPUT);
  pinMode(PIN_LOGAS , OUTPUT);
  pinMode(PIN_FUEL1 , OUTPUT);
  pinMode(PIN_HIGAS , OUTPUT);
  
  // Print header and instructions
  Serial.println(F("Burner Control System Simulation"));
  Serial.println(F("--------------------------------"));
  Serial.println(F("Commands:"));
  Serial.println(F("  start - Press start button"));
  Serial.println(F("  stop - Press stop button"));
  Serial.println(F("  pcfs:on/off - Set Primary Combustion Flame Sensor"));
  Serial.println(F("  scfs:on/off - Set Secondary Combustion Flame Sensor"));
  Serial.println(F("  lts:value - Set Load Temperature Sensor value"));
  Serial.println(F("  fts:value - Set Flame Temperature Sensor value"));
  Serial.println(F("  status - Show current system status"));
  Serial.println(F("--------------------------------"));
}
void loop() {
  // ---- Real-world inputs ----
  MSTART = (digitalRead(PIN_MSTART) == LOW);   // buttons active LOW
  MSTOP  = (digitalRead(PIN_MSTOP ) == LOW);
  PCFS   = (digitalRead(PIN_PCFS ) == LOW);
  SCFS   = (digitalRead(PIN_SCFS ) == LOW);
  LTS = analogRead(PIN_LTS);   // 0-1023 scale for now
  FTS = analogRead(PIN_FTS);
  // Read serial input
  readSerialInput();
  
  // Process commands if available
  if (serial_complete) {
    processCommand();
    serial_complete = false;
    serial_input = "";
  }
  
  // Run state machine before resetting momentary buttons
  runStateMachine();
  // ---- Drive LEDs / relays ----
  digitalWrite(PIN_BLOW  , BLOW);
  digitalWrite(PIN_SHAFT , SHAFT);
  digitalWrite(PIN_IGNITE, IGNITE);
  digitalWrite(PIN_LOGAS , LOGAS);
  digitalWrite(PIN_FUEL1 , FUEL1);
  digitalWrite(PIN_HIGAS , HIGAS);
  
  // Update timers and reset momentary buttons
  updateTimers();
  
  // Display status periodically (every second)
  static unsigned long last_status = 0;
  if (millis() - last_status > 1000) {
    displayStatus();
    last_status = millis();
  }
}
// Read and process serial input
void readSerialInput() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '') {
      serial_complete = true;
    } else {
      serial_input += inChar;
    }
  }
}
// Process commands from serial input
void processCommand() {
  serial_input.trim();
  
  if (serial_input.equals("start")) {
    MSTART = true;
    Serial.println(F("START button pressed"));
  } 
  else if (serial_input.equals("stop")) {
    MSTOP = true;
    Serial.println(F("STOP button pressed"));
  }
  else if (serial_input.equals("pcfs:on")) {
    PCFS = true;
    Serial.println(F("Primary Combustion Flame Sensor: ON"));
  }
  else if (serial_input.equals("pcfs:off")) {
    PCFS = false;
    Serial.println(F("Primary Combustion Flame Sensor: OFF"));
  }
  else if (serial_input.equals("scfs:on")) {
    SCFS = true;
    Serial.println(F("Secondary Combustion Flame Sensor: ON"));
  }
  else if (serial_input.equals("scfs:off")) {
    SCFS = false;
    Serial.println(F("Secondary Combustion Flame Sensor: OFF"));
  }
  else if (serial_input.startsWith("lts:")) {
    LTS = serial_input.substring(4).toInt();
    Serial.print(F("Load Temperature Sensor set to: "));
    Serial.println(LTS);
  }
  else if (serial_input.startsWith("fts:")) {
    FTS = serial_input.substring(4).toInt();
    Serial.print(F("Flame Temperature Sensor set to: "));
    Serial.println(FTS);
  }
  else if (serial_input.equals("status")) {
    displayDetailedStatus();
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(serial_input);
  }
}
// Start a timer with specified duration
void startTimer(unsigned long duration) {
  timer_start = millis();
  timer_duration = duration;
  timer_running = true;
}
// Check if timer has expired
bool isTimerExpired() {
  if (!timer_running) {
    return false;
  }
  
  current_time = millis();
  if (current_time - timer_start >= timer_duration) {
    timer_running = false;
    return true;
  }
  
  return false;
}
// Update timers and handle button presses
void updateTimers() {
   // ----- legacy serial-only momentary buttons -----
  // Keep this section *only* if you still want typed "start" / "stop"
  // to behave like a quick tap.
  /*
  if (MSTART) MSTART = false;
  if (MSTOP ) MSTOP  = false;
  */
}
// Display current system status
void displayStatus() {
  sprintf(
    serial_buffer,
    "State: %-15s | BLOW: %-3s | SHAFT: %-3s | IGNITE: %-3s | LOGAS: %-3s | HIGAS: %-3s | FUEL1: %-3s | IFFI: %-3s | SCFI: %-3s",
    state_strings[current_state],
    BLOW ? "ON" : "OFF",
    SHAFT ? "ON" : "OFF",
    IGNITE ? "ON" : "OFF",
    LOGAS ? "ON" : "OFF",
    HIGAS ? "ON" : "OFF",
    FUEL1 ? "ON" : "OFF",
    IFFI ? "ON" : "OFF",
    SCFI ? "ON" : "OFF"
  );
  Serial.println(serial_buffer);
  
  // Display timer information if a timer is running
  if (timer_running) {
    unsigned long remaining = timer_duration - (millis() - timer_start);
    Serial.print(F("Timer: "));
    Serial.print(remaining / 1000);
    Serial.println(F(" seconds remaining"));
  }
}
// Display detailed system status
void displayDetailedStatus() {
  Serial.println(F("--- SYSTEM STATUS ---"));
  Serial.print(F("Current State: "));
  Serial.println(state_strings[current_state]);
  
  Serial.println(F("INPUTS:"));
  Serial.print(F("MSTART (Momentary Start Switch): "));
  Serial.println(MSTART ? "ON" : "OFF");
  Serial.print(F("MSTOP (Momentary Stop Switch): "));
  Serial.println(MSTOP ? "ON" : "OFF");
  Serial.print(F("PCFS (Primary Combustion Flame Sensor): "));
  Serial.println(PCFS ? "ON" : "OFF");
  Serial.print(F("SCFS (Secondary Combustion Flame Sensor): "));
  Serial.println(SCFS ? "ON" : "OFF");
  Serial.print(F("LTS (Load Temperature Sensor): "));
  Serial.println(LTS);
  Serial.print(F("FTS (Flame Temperature Sensor): "));
  Serial.println(FTS);
  
  Serial.println(F("OUTPUTS:"));
  Serial.print(F("BLOW (Combustion Blower Power): "));
  Serial.println(BLOW ? "ON" : "OFF");
  Serial.print(F("SHAFT (Main Shaft Power): "));
  Serial.println(SHAFT ? "ON" : "OFF");
  Serial.print(F("IGNITE (Hot Surface Ignitor Power): "));
  Serial.println(IGNITE ? "ON" : "OFF");
  Serial.print(F("LOGAS (Preheat fuel gas low rate): "));
  Serial.println(LOGAS ? "ON" : "OFF");
  Serial.print(F("HIGAS (Preheat fuel gas high rate): "));
  Serial.println(HIGAS ? "ON" : "OFF");
  Serial.print(F("FUEL1 (Main Fuel 1): "));
  Serial.println(FUEL1 ? "ON" : "OFF");
  Serial.print(F("FUEL2 (Main Fuel 2): "));
  Serial.println(FUEL2 ? "ON" : "OFF");
  Serial.print(F("FUEL3 (Main Fuel 3): "));
  Serial.println(FUEL3 ? "ON" : "OFF");
  Serial.print(F("H2O (Cleaning Steam/Water Supply): "));
  Serial.println(H2O ? "ON" : "OFF");
  Serial.print(F("IFFI (Ignition Failure Fault Indicator): "));
  Serial.println(IFFI ? "ON" : "OFF");
  Serial.print(F("SCFI (Secondary Combustion Fault Indicator): "));
  Serial.println(SCFI ? "ON" : "OFF");
  
  Serial.println(F("COUNTERS:"));
  Serial.print(F("PCFC (Primary Combustion Flame Sensor Count): "));
  Serial.println(PCFC);
  Serial.print(F("SCFC (Secondary Combustion Flame Sensor Count): "));
  Serial.println(SCFC);
  
  Serial.println(F("-------------------"));
}
// Main state machine implementation
void runStateMachine() {
  switch (current_state) {
    case STOPPED:
      // In STOPPED state, all outputs are off
      BLOW = false;
      SHAFT = false;
      IGNITE = false;
      LOGAS = false;
      HIGAS = false;
      FUEL1 = false;
      FUEL2 = false;
      FUEL3 = false;
      H2O = false;
      IFFI = false;
      SCFI = false;
      
      // Reset counters
      PCFC = 0;
      SCFC = 0;
      
      // Check for start button press
      if (MSTART) {
        current_state = START_SEQUENCE;
        Serial.println(F("Starting burner sequence..."));
      }
      break;
      
    case START_SEQUENCE:
      // Start the purge sequence
      BLOW = true;  // Turn on blower
      
      // Start purge air delay timer
      startTimer(PADT);
      current_state = PURGE;
      Serial.println(F("Purge sequence started"));
      break;
      
    case PURGE:
      // Wait for purge timer to complete
      if (isTimerExpired()) {
        // Purge complete, start shaft and ignition
        SHAFT = true;
        IGNITE = true;
        
        // Start hot surface ignitor warm-up timer
        startTimer(HSOT);
        Serial.println(F("Purge complete, starting ignition sequence"));
        current_state = IGNITION;
      }
      
      // Check for stop button press
      if (MSTOP) {
        current_state = STOP_SEQUENCE;
      }
      break;
      
    case IGNITION:
      // Check if hot surface ignitor warm-up is complete
      if (isTimerExpired()) {
        // Turn on high gas for ignition
        HIGAS = true;
        
        // Start primary combustion safety timer
        startTimer(PCST);
        Serial.println(F("Ignitor warm-up complete, starting fuel"));
      }
      
      // Check for flame detection
      if (PCFS) {
        // Reset flame counter when flame is detected
        PCFC = 0;
        
        // If flame is established, move to pre-heat delay
        if (!timer_running) {
          HIGAS = false;  // Turn off high gas
          LOGAS = true;   // Turn on low gas for pre-heat
          
          // Start pre-heat delay timer
          startTimer(PHDT);
          Serial.println(F("Flame detected, starting pre-heat"));
        }
      } else if (!timer_running) {
        // If no flame and no timer running, increment fault counter
        PCFC++;
        
        // If too many failures, go to fault
        if (PCFC >= 3) {
          IFFI = true;  // Set ignition failure indicator
          IGNITE = false;
          HIGAS = false;
          current_state = FAULT;
          Serial.println(F("FAULT: Ignition failure after 3 attempts"));
        } else {
          // Retry ignition
          startTimer(PCST);
          Serial.print(F("Ignition retry attempt "));
          Serial.println(PCFC);
        }
      }
      
      // Check for safety timer expiration (no flame detected)
      if (timer_running && isTimerExpired() && !PCFS) {
        // If safety timer expires without flame, increment fault counter
        PCFC++;
        
        // If too many failures, go to fault
        if (PCFC >= 3) {
          IFFI = true;  // Set ignition failure indicator
          IGNITE = false;
          HIGAS = false;
          current_state = FAULT;
          Serial.println(F("FAULT: Ignition failure after 3 attempts"));
        } else {
          // Retry ignition
          startTimer(PCST);
          Serial.print(F("Ignition retry attempt "));
          Serial.println(PCFC);
        }
      }
      
      // If pre-heat timer expires, move to running state
      if (!timer_running && LOGAS) {
        LOGAS = false;
        FUEL1 = true;
        
        // Start main fuel delay timer
        startTimer(MFDT);
        Serial.println(F("Pre-heat complete, starting main fuel"));
        current_state = RUNNING;
      }
      
      // Check for stop button press
      if (MSTOP) {
        current_state = STOP_SEQUENCE;
      }
      break;
      
    case RUNNING:
      // Check if main fuel delay timer has expired
      if (isTimerExpired()) {
        // Turn off ignition after main fuel is established
        IGNITE = false;
        Serial.println(F("Main fuel established, system running"));
      }
      
      // Continuous monitoring of flame sensors
      if (!PCFS) {
        // Start primary combustion safety timer if flame is lost
        if (!timer_running) {
          startTimer(PCST);
          Serial.println(F("WARNING: Primary flame lost, safety timer started"));
        }
      } else {
        // Reset timer if flame is detected again
        timer_running = false;
      }
      
      // Check for secondary flame if required
      if (FUEL1 && !SCFS) {
        // Increment secondary flame fault counter
        SCFC++;
        
        if (SCFC > 3) {
          SCFI = true;
          current_state = FAULT;
          Serial.println(F("FAULT: Secondary combustion failure"));
        }
      } else {
        // Reset counter if flame is detected
        SCFC = 0;
      }
      
      // Check for safety timer expiration
      if (timer_running && isTimerExpired() && !PCFS) {
        IFFI = true;
        current_state = FAULT;
        Serial.println(F("FAULT: Primary flame lost during operation"));
      }
      
      // Check for stop button press
      if (MSTOP) {
        current_state = STOP_SEQUENCE;
      }
      break;
      
    case STOP_SEQUENCE:
      // Turn off all fuel first
      FUEL1 = false;
      FUEL2 = false;
      FUEL3 = false;
      LOGAS = false;
      HIGAS = false;
      
      // Turn off ignition
      IGNITE = false;
      
      // Turn off shaft
      SHAFT = false;
      
      // Keep blower on for purge
      startTimer(PADT);
      Serial.println(F("Stop sequence initiated, purging system"));
      
      // Wait for purge timer to complete
      if (isTimerExpired()) {
        // Turn off blower
        BLOW = false;
        current_state = STOPPED;
        Serial.println(F("System stopped"));
      }
      break;
      
    case FAULT:
      // In fault state, all outputs except fault indicators are off
      BLOW = false;
      SHAFT = false;
      IGNITE = false;
      LOGAS = false;
      HIGAS = false;
      FUEL1 = false;
      FUEL2 = false;
      FUEL3 = false;
      H2O = false;
      
      // Display fault information
      Serial.println(F("SYSTEM IN FAULT STATE"));
      if (IFFI) {
        Serial.println(F("Ignition Failure Fault"));
      }
      if (SCFI) {
        Serial.println(F("Secondary Combustion Fault"));
      }
      
      // Check for reset (both start and stop pressed)
      if (MSTART && MSTOP) {
        // Reset fault indicators
        IFFI = false;
        SCFI = false;
        current_state = STOPPED;
        Serial.println(F("Fault reset, system stopped"));
      }
      break;
  }
}
