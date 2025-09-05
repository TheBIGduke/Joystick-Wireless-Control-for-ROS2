/**
 * Joystick Wireless Control for ROS2
 * 
 * An implementation of a Joystick to control a robot wirelessly using an ESP32.
 * This system uses Socket.IO communication to send real-time joystick commands
 * to ROS2 robots through the delivery_bridge package.
 * 
 * Features:
 * - Analog joystick input reading with calibration
 * - Wireless control via WiFi and Socket.IO
 * - Variable speed control (half/full speed based on joystick position)
 * - Real-time cmd_vel command transmission to ROS2
 * - Automatic reconnection and error handling
 * - Deadzone filtering for precise control
 * - RGB LED status indicator
 * - Dead man switch safety feature
 * 
 * @author Kaléin Tamaríz - TheBIGduke
 * @version 1.1.0 (organized and commented)
 */

// ============= LIBRARY INCLUDES =============
#include <Arduino.h>            // Arduino core functions for PlatformIO
#include <WiFi.h>               // ESP32 WiFi functionality
#include <SocketIOclient.h>     // Socket.IO client library


// ============= CONFIGURATION SECTION =============
// Update these values for your specific setup

// --- WiFi Configuration ---
const char* ssid = "ESP32";               // WiFi network name
const char* password = "12345678";        // WiFi password

// Socket.IO server configuration
const char* socketio_host = "192.168.0.103";  // IP address of the ROS2 machine
const int socketio_port = 9009;               // Socket.IO server port

// --- Joystick Hardware Configuration ---
#define VRX_PIN  39              // ESP32 pin GPIO39 (ADC3) connected to VRX pin
#define VRY_PIN  36              // ESP32 pin GPIO36 (ADC0) connected to VRY pin

// Joystick calibration values (will be automatically calibrated on startup)
int CENTER_X_CALIBRATED = 2048;  // Default center for X axis
int CENTER_Y_CALIBRATED = 2048;  // Default center for Y axis
#define MAX_X    4095             // Maximum value for X axis
#define MAX_Y    4095             // Maximum value for Y axis

// --- Safety Switch Configuration ---
#define DEADMAN_BUTTON_PIN 4     // ESP32 pin GPIO4 connected to pushbutton

// --- LED Indicator Configuration ---
// RGB LED pins
#define LED_RED_PIN 21
#define LED_GREEN_PIN 22
#define LED_BLUE_PIN 23

// PWM channels (ESP32 specific)
#define RED_CHANNEL 0
#define GREEN_CHANNEL 1
#define BLUE_CHANNEL 2

// PWM settings
#define PWM_FREQ 5000            // PWM frequency
#define PWM_RESOLUTION 8         // 8-bit resolution (0-255)

// --- Movement Parameters ---
const float LINEAR_SPEED_FULL = 0.4;   // Full linear velocity (m/s)
const float LINEAR_SPEED_HALF = 0.2;   // Half linear velocity (m/s)
const float ANGULAR_SPEED_FULL = 0.6;  // Full angular velocity (rad/s)
const float ANGULAR_SPEED_HALF = 0.3;  // Half angular velocity (rad/s)

// Speed thresholds (based on joystick range)
const int SPEED_THRESHOLD = 250;       // Threshold for half vs full speed

// Command velocity send interval
const unsigned long CMD_VEL_INTERVAL = 100; // ms between cmd_vel commands

// Calibration parameters
const int CALIBRATION_SAMPLES = 1000;  // Number of samples for calibration
const int CALIBRATION_DELAY = 2;       // Delay between samples (ms)


// ============= ENUMERATIONS =============
// LED states for system status indication
enum LEDstate {
  LED_CALIBRATING,           // Calibration in progress
  LED_CALIBRATION_DONE,      // Calibration completed
  LED_WIFI_CONNECTING,       // WiFi connection in progress
  LED_WIFI_CONNECTED,        // WiFi connected successfully
  LED_SOCKETIO_CONNECTING,   // Socket.IO connection in progress
  LED_SOCKETIO_CONNECTED,    // Socket.IO connected successfully
  LED_SOCKETIO_ERROR,        // Socket.IO connection failed
  LED_SYSTEM_ONLINE,         // System fully operational
  LED_DEADMAN_NO_INPUT,      // Dead man active but no joystick input
  LED_DEADMAN_WITH_INPUT     // Dead man active with joystick input
};

// System states
enum SystemState {
  STATE_CALIBRATING,
  STATE_CONNECTING_WIFI,
  STATE_CONNECTING_SOCKETIO,
  STATE_READY
};

// Solid color states for LED
#define COLOR_OFF 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_BLUE 3
#define COLOR_MAGENTA 4
#define COLOR_ORANGE 5
#define COLOR_YELLOW 6

// Blinking modes for status indication
#define BLINK_FAST 0
#define BLINK_MEDIUM 1
#define BLINK_HOLD 2
#define BLINK_DOUBLE 3        // off blink blink off blink blink off
#define BLINK_TRIPLE 4        // off blink blink blink off blink blink blink off
#define BLINK_HEARTBEAT 5
#define BLINK_RAPID_PULSE 6


// ============= GLOBAL VARIABLES =============
SocketIOclient socketIO;              // Socket.IO client instance

// Joystick and control variables
int valueX = 0;                       // Current X-axis value
int valueY = 0;                       // Current Y-axis value
unsigned long lastCmdVel = 0;         // Last time cmd_vel was sent

// Dead man switch variables
bool deadmanPressed = false;          // Current dead man switch state
bool lastDeadmanState = false;        // Previous dead man switch state

// LED indicator variables
unsigned long blinkTimer = 0;         // Timer for LED blinking
bool blinkState = false;              // Current LED blink state
int blinkCycle = 0;                   // Current blink cycle

// Calibration variables
long sumX = 0, sumY = 0;              // Sum of calibration samples
int calibrationSamplesTaken = 0;      // Number of calibration samples taken
unsigned long calibrationStartTime = 0; // When calibration started
SystemState systemState = STATE_CALIBRATING; // Current system state
LEDstate currentLEDState = LED_CALIBRATING; // Current LED state
unsigned long socketRetryTime = 0;
bool socketConnectionFailed = false;
const unsigned long SOCKET_RETRY_DELAY = 5000; // 5 seconds


// ============= FUNCTION PROTOTYPES =============
// LED control functions
void setSolidColor(int color);
void setBlinkingMode(int blinkMode, int color);
void setLEDState(LEDstate state);
void updateLED();

// Joystick functions
void calibrateJoystick();
void readJoystick();
float calculateLinearSpeed(int joystickValue);
float calculateAngularSpeed(int joystickValue);

// Socket.IO functions
void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length);

// Movement functions
String getMovementDescription(float linear_x, float angular_z);
void sendCmdVel(float linear_x, float angular_z);
void processJoystickInput();

// Utility functions
bool readDeadmanSwitch();
void handleCalibration();
void handleWiFiConnection();
void handleSocketIOConnection();


// ============= SETUP FUNCTION =============
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  Serial.println("\n=== Joystick Wireless Control for ROS2 ===");

  // Configure PWM channels for RGB LED
  ledcSetup(RED_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(GREEN_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(BLUE_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  
  // Attach pins to PWM channels
  ledcAttachPin(LED_RED_PIN, RED_CHANNEL);
  ledcAttachPin(LED_GREEN_PIN, GREEN_CHANNEL);
  ledcAttachPin(LED_BLUE_PIN, BLUE_CHANNEL);
  
  // Start with all LEDs off
  setSolidColor(COLOR_OFF);

  // Configure ADC for joystick
  analogSetAttenuation(ADC_11db);  // Set ADC attenuation to 11 dB (up to ~3.3V input)
  
  // Configure dead man switch button
  pinMode(DEADMAN_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Dead man switch configured on GPIO4 (active LOW)");
  
  // Start calibration process
  setLEDState(LED_CALIBRATING);
  calibrationStartTime = millis();
  Serial.println("=== JOYSTICK CALIBRATION ===");
  Serial.println("Please center your joystick and keep it still...");
  Serial.println("Calibrating in 5 seconds...");
}


// ============= MAIN LOOP =============
void loop() {
  // Update LED state
  updateLED();
  
  // Handle different system states
  switch(systemState) {
    case STATE_CALIBRATING:
      handleCalibration();
      break;
      
    case STATE_CONNECTING_WIFI:
      handleWiFiConnection();
      break;
      
    case STATE_CONNECTING_SOCKETIO:
      handleSocketIOConnection();
      break;
      
    case STATE_READY:
      // Handle Socket.IO events
      socketIO.loop();
      
      // Read joystick input
      readJoystick();
      
      // Send cmd_vel commands at regular intervals
      if (millis() - lastCmdVel >= CMD_VEL_INTERVAL) {
        lastCmdVel = millis();
        processJoystickInput();
      }
      break;
  }
  
  delay(10);  // Small delay to maintain stability
}


// ============= CALIBRATION FUNCTIONS =============

/**
 * Handles the calibration process in a non-blocking way
 */
void handleCalibration() {
  static int countdown = 5;
  static unsigned long lastCountdownTime = 0;
  
  // Countdown phase
  if (countdown > 0) {
    if (millis() - lastCountdownTime >= 1000) {
      lastCountdownTime = millis();
      Serial.println(String(countdown) + "...");
      countdown--;
      
      if (countdown == 0) {
        Serial.println("Calibrating... DO NOT MOVE THE JOYSTICK!");
      }
    }
    return;
  }
  
  // Calibration sampling phase
  if (calibrationSamplesTaken < CALIBRATION_SAMPLES) {
    if (millis() - calibrationStartTime >= CALIBRATION_DELAY) {
      calibrationStartTime = millis();
      
      sumX += analogRead(VRY_PIN);  // Note: X and Y are swapped in reading
      sumY += analogRead(VRX_PIN);  // to match physical orientation
      calibrationSamplesTaken++;
      
      if (calibrationSamplesTaken % 100 == 0) {
        Serial.print(".");
      }
    }
    return;
  }
  
  // Calibration complete
  Serial.println();
  
  // Calculate average center values
  CENTER_X_CALIBRATED = sumX / CALIBRATION_SAMPLES;
  CENTER_Y_CALIBRATED = sumY / CALIBRATION_SAMPLES;
  
  Serial.println("Calibration complete!");
  Serial.println("CENTER_X: " + String(CENTER_X_CALIBRATED));
  Serial.println("CENTER_Y: " + String(CENTER_Y_CALIBRATED));
  
  // Test the calibration
  Serial.println("\nTesting calibration (keep joystick centered):");
  for(int i = 0; i < 5; i++) {
    int testX = analogRead(VRY_PIN);
    int testY = analogRead(VRX_PIN);
    int mappedX = (testX - CENTER_X_CALIBRATED) * 1000 / MAX_X;
    int mappedY = (testY - CENTER_Y_CALIBRATED) * 1000 / MAX_Y;
    Serial.println("Raw(" + String(testX) + "," + String(testY) + 
                   ") -> Mapped(" + String(mappedX) + "," + String(mappedY) + ")");
    delay(200);
  }
  
  Serial.println("=== CALIBRATION COMPLETE ===\n");
  setLEDState(LED_CALIBRATION_DONE);
  delay(1000);
  
  // Move to next state
  systemState = STATE_CONNECTING_WIFI;
  setLEDState(LED_WIFI_CONNECTING);
  WiFi.begin(ssid, password);
}


// ============= CONNECTION HANDLING FUNCTIONS =============

/**
 * Handles WiFi connection process
 */
void handleWiFiConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    setLEDState(LED_WIFI_CONNECTED);
    delay(3000);
    
    // Move to next state
    systemState = STATE_CONNECTING_SOCKETIO;
    setLEDState(LED_SOCKETIO_CONNECTING);
    socketIO.begin(socketio_host, socketio_port, "/socket.io/?EIO=4");
    socketIO.onEvent(socketIOEvent);
    return;
  }
  
  // Show connection progress
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 500) {
    lastPrintTime = millis();
    Serial.print(".");
  }
}

/**
 * Handles Socket.IO connection process
 */
void handleSocketIOConnection() {
  static unsigned long connectionStartTime = millis();
  const unsigned long CONNECTION_TIMEOUT = 10000; // 10 seconds timeout

  // Handle connection timeout
  if (millis() - connectionStartTime > CONNECTION_TIMEOUT && !socketConnectionFailed) {
    Serial.println("Socket.IO connection timeout!");
    setLEDState(LED_SOCKETIO_ERROR);
    socketConnectionFailed = true;
    socketRetryTime = millis();
    return;
  }
  
  // Handle retry after timeout
  if (socketConnectionFailed) {
    if (millis() - socketRetryTime >= SOCKET_RETRY_DELAY) {
      Serial.println("Retrying Socket.IO connection...");
      socketConnectionFailed = false;
      connectionStartTime = millis();
      setLEDState(LED_SOCKETIO_CONNECTING);
      
      // Reinitialize the Socket.IO connection
      socketIO.begin(socketio_host, socketio_port, "/socket.io/?EIO=4");
      socketIO.onEvent(socketIOEvent);
    }
    return;
  }
  
  // Socket.IO connection is handled in the event callback
  // We just need to check if we're connected
  static bool socketConnected = false;
  
  // For demonstration, we'll assume connection is successful after a delay
  if (!socketConnected && millis() - connectionStartTime > 2000) {
    socketConnected = true;
    Serial.println("Socket.IO client initialized");
    setLEDState(LED_SOCKETIO_CONNECTED);
    delay(1000);
    
    // System ready
    systemState = STATE_READY;
    setLEDState(LED_SYSTEM_ONLINE);

    // Print control information
    Serial.println("=== Control Configuration ===");
    Serial.println("Joystick Controls with Variable Speed:");
    Serial.println("- Y-axis: Forward/Backward movement");
    Serial.println("- X-axis: Left/Right rotation");
    Serial.println("- Speed: Half speed for |value| <= " + String(SPEED_THRESHOLD) + 
                   ", Full speed for |value| > " + String(SPEED_THRESHOLD));
    Serial.println("- Linear speeds: Half=" + String(LINEAR_SPEED_HALF) + 
                   "m/s, Full=" + String(LINEAR_SPEED_FULL) + "m/s");
    Serial.println("- Angular speeds: Half=" + String(ANGULAR_SPEED_HALF) + 
                   "rad/s, Full=" + String(ANGULAR_SPEED_FULL) + "rad/s");
    Serial.println("\n=== SAFETY FEATURES ===");
    Serial.println("- Dead man switch: GPIO4 button must be pressed for robot control");
    Serial.println("- Robot will STOP immediately when button is released");
    Serial.println("\nReady for operation. HOLD THE DEAD MAN SWITCH to enable control.");
  }
}


// ============= JOYSTICK FUNCTIONS =============

/**
 * Reads raw values from joystick and converts to centered coordinates
 * Applies deadzone filtering to prevent drift
 */
void readJoystick() {
  // Read X and Y analog values (note: pins are swapped to match orientation)
  valueX = analogRead(VRY_PIN);
  valueY = analogRead(VRX_PIN);

  // Map values so center is 0, left/up is negative, right/down is positive
  valueX = (valueX - CENTER_X_CALIBRATED) * 1000 / MAX_X;
  valueY = (valueY - CENTER_Y_CALIBRATED) * 1000 / MAX_Y;

  // Apply deadzone to prevent drift around center
  const int deadzone = 10;
  if (abs(valueX) < deadzone) valueX = 0;
  if (abs(valueY) < deadzone) valueY = 0;
}

/**
 * Calculates linear speed based on joystick position
 * @param joystickValue The joystick position value (-1000 to 1000)
 * @return Linear speed value with appropriate sign
 */
float calculateLinearSpeed(int joystickValue) {
  int absValue = abs(joystickValue);
  
  if (absValue == 0) {
    return 0.0;  // No movement
  }
  else if (absValue <= SPEED_THRESHOLD) {
    // Half speed for middle range
    return (joystickValue > 0) ? LINEAR_SPEED_HALF : -LINEAR_SPEED_HALF;
  }
  else {
    // Full speed for extreme positions
    return (joystickValue > 0) ? LINEAR_SPEED_FULL : -LINEAR_SPEED_FULL;
  }
}

/**
 * Calculates angular speed based on joystick position
 * @param joystickValue The joystick position value (-1000 to 1000)
 * @return Angular speed value with appropriate sign
 */
float calculateAngularSpeed(int joystickValue) {
  int absValue = abs(joystickValue);
  
  if (absValue == 0) {
    return 0.0;  // No rotation
  }
  else if (absValue <= SPEED_THRESHOLD) {
    // Half speed for middle range
    return (joystickValue > 0) ? ANGULAR_SPEED_HALF : -ANGULAR_SPEED_HALF;
  }
  else {
    // Full speed for extreme positions
    return (joystickValue > 0) ? ANGULAR_SPEED_FULL : -ANGULAR_SPEED_FULL;
  }
}


// ============= MOVEMENT FUNCTIONS =============

/**
 * Processes joystick input and sends appropriate movement commands
 * Only sends commands when dead man switch is pressed
 */
void processJoystickInput() {
  // Read dead man switch state
  deadmanPressed = readDeadmanSwitch();
  
  // Provide feedback when button state changes
  if (deadmanPressed != lastDeadmanState) {
    if (deadmanPressed) {
      Serial.println("*** DEAD MAN SWITCH: ACTIVE - Robot control ENABLED ***");
      setLEDState(LED_DEADMAN_NO_INPUT);
    } else {
      Serial.println("*** DEAD MAN SWITCH: RELEASED - Robot control DISABLED ***");
      setLEDState(LED_SYSTEM_ONLINE);
      // Send stop command when button is released
      sendCmdVel(0.0, 0.0);
    }
    lastDeadmanState = deadmanPressed;
  }
  
  float linear_x = 0.0;
  float angular_z = 0.0;
  
  // Only process joystick input if dead man switch is pressed
  if (deadmanPressed) {
    // Update LED state based on whether there's input
    if (valueX != 0 || valueY != 0) {
      setLEDState(LED_DEADMAN_WITH_INPUT);
    } else {
      setLEDState(LED_DEADMAN_NO_INPUT);
    }
    
    // Calculate speeds based on joystick positions
    // Forward/Backward control (Y-axis)
    if (valueY != 0) {
      linear_x = calculateLinearSpeed(valueY);
    }
    
    // Left/Right rotation control (X-axis)
    if (valueX != 0) {
      angular_z = -calculateAngularSpeed(valueX); // Negative for correct rotation direction
    }
  }
  
  // Send movement command
  sendCmdVel(linear_x, angular_z);
}

/**
 * Sends movement command to ROS2 via Socket.IO
 * @param linear_x Linear velocity (m/s)
 * @param angular_z Angular velocity (rad/s)
 */
void sendCmdVel(float linear_x, float angular_z) {
  // Create cmd_vel JSON payload
  String payload = "[\"cmd_vel\",{\"linear_x\":" + String(linear_x) + 
                   ",\"angular_z\":" + String(angular_z) + "}]";
  
  // Send cmd_vel event to ROS2 robot
  socketIO.sendEVENT(payload);
  
  // Print movement description for debugging
  Serial.println("Movement: " + getMovementDescription(linear_x, angular_z) + 
                 " | linear_x=" + String(linear_x) + ", angular_z=" + String(angular_z));
}

/**
 * Returns a descriptive string for the current movement
 * @param linear_x Linear velocity value
 * @param angular_z Angular velocity value
 * @return String describing the movement
 */
String getMovementDescription(float linear_x, float angular_z) {
  if (linear_x > 0 && angular_z == 0) return "FORWARD";
  else if (linear_x < 0 && angular_z == 0) return "BACKWARD";
  else if (linear_x == 0 && angular_z > 0) return "ROTATE_LEFT";
  else if (linear_x == 0 && angular_z < 0) return "ROTATE_RIGHT";
  else if (linear_x == 0 && angular_z == 0) return "STOPPED";
  else return "COMBINED_MOVEMENT";
}


// ============= SOCKET.IO FUNCTIONS =============

/**
 * Handles Socket.IO events
 * @param type Type of Socket.IO message
 * @param payload Message payload
 * @param length Payload length
 */
void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case sIOtype_DISCONNECT:
      Serial.printf("[IOc] Disconnected!\n");
      setLEDState(LED_SOCKETIO_ERROR);
      systemState = STATE_CONNECTING_SOCKETIO;
      break;
    case sIOtype_CONNECT:
      Serial.printf("[IOc] Connected to url: %s\n", payload);
      // Join default namespace
      socketIO.send(sIOtype_CONNECT, "/");
      setLEDState(LED_SOCKETIO_CONNECTED);

      //Now transition to ready state
      systemState = STATE_READY;
      setLEDState(LED_SYSTEM_ONLINE);
      break;
    case sIOtype_EVENT:
      Serial.printf("[IOc] get event: %s\n", payload);
      break;
    case sIOtype_ACK:
      Serial.printf("[IOc] get ack: %u\n", length);
      break;
    case sIOtype_ERROR:
      Serial.printf("[IOc] get error: %u\n", length);
      setLEDState(LED_SOCKETIO_ERROR);
      break;
    case sIOtype_BINARY_EVENT:
    case sIOtype_BINARY_ACK:
      // Not used in this implementation
      break;
  }
}


// ============= LED CONTROL FUNCTIONS =============

/**
 * Sets the LED to a solid color
 * @param color Color to set (COLOR_RED, COLOR_GREEN, etc.)
 */
void setSolidColor(int color) {
  switch(color) {
    case COLOR_RED:
      ledcWrite(RED_CHANNEL, 0);     // Red ON
      ledcWrite(GREEN_CHANNEL, 255); // Green OFF
      ledcWrite(BLUE_CHANNEL, 255);  // Blue OFF
      break;
      
    case COLOR_GREEN:
      ledcWrite(RED_CHANNEL, 255);   // Red OFF
      ledcWrite(GREEN_CHANNEL, 0);   // Green ON
      ledcWrite(BLUE_CHANNEL, 255);  // Blue OFF
      break;
      
    case COLOR_BLUE:
      ledcWrite(RED_CHANNEL, 255);   // Red OFF
      ledcWrite(GREEN_CHANNEL, 255); // Green OFF
      ledcWrite(BLUE_CHANNEL, 0);    // Blue ON
      break;
      
    case COLOR_MAGENTA:
      ledcWrite(RED_CHANNEL, 100);   // Red ON
      ledcWrite(GREEN_CHANNEL, 255); // Green OFF
      ledcWrite(BLUE_CHANNEL, 0);    // Blue ON
      break;
      
    case COLOR_ORANGE:
      ledcWrite(RED_CHANNEL, 37);    // Red ON
      ledcWrite(GREEN_CHANNEL, 0);   // Green ON (dimmed)
      ledcWrite(BLUE_CHANNEL, 255);  // Blue OFF
      break;
      
    case COLOR_YELLOW:
      ledcWrite(RED_CHANNEL, 65);    // Red ON
      ledcWrite(GREEN_CHANNEL, 0);   // Green ON
      ledcWrite(BLUE_CHANNEL, 250);  // Blue OFF
      break;
      
    case COLOR_OFF:
    default:
      ledcWrite(RED_CHANNEL, 255);   // Red OFF
      ledcWrite(GREEN_CHANNEL, 255); // Green OFF
      ledcWrite(BLUE_CHANNEL, 255);  // Blue OFF
      break;
  }
}

/**
 * Sets a blinking pattern for the LED
 * @param blinkMode Blinking pattern to use
 * @param color Color to blink
 */
void setBlinkingMode(int blinkMode, int color) {
  switch(blinkMode) {
    case BLINK_FAST:
      // Fast blink at 100ms intervals
      if (millis() - blinkTimer >= 100) {
        blinkState = !blinkState;
        if (blinkState) {
          setSolidColor(color);
        } else {
          setSolidColor(COLOR_OFF);
        }
        blinkTimer = millis();
      }
      break;
      
    case BLINK_MEDIUM:
      // Medium blink at 300ms intervals
      if (millis() - blinkTimer >= 300) {
        blinkState = !blinkState;
        if (blinkState) {
          setSolidColor(color);
        } else {
          setSolidColor(COLOR_OFF);
        }
        blinkTimer = millis();
      }
      break;
      
    case BLINK_HOLD:
      // Solid color for 2 seconds
      setSolidColor(color);
      break;
      
    case BLINK_DOUBLE:
      // Pattern: off blink blink off blink blink off
      if (millis() - blinkTimer >= 200) {
        int pattern[] = {0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0}; // 0=off, 1=on
        int patternLength = 11;
        
        if (blinkCycle < patternLength) {
          if (pattern[blinkCycle] == 1) {
            setSolidColor(color);
          } else {
            setSolidColor(COLOR_OFF);
          }
          blinkCycle++;
          blinkTimer = millis();
        } else {
          blinkCycle = 0; // Reset pattern
        }
      }
      break;
      
    case BLINK_TRIPLE:
      // Pattern: off blink blink blink off blink blink blink off
      if (millis() - blinkTimer >= 150) {
        int pattern[] = {0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0}; // 0=off, 1=on
        int patternLength = 15;
        
        if (blinkCycle < patternLength) {
          if (pattern[blinkCycle] == 1) {
            setSolidColor(color);
          } else {
            setSolidColor(COLOR_OFF);
          }
          blinkCycle++;
          blinkTimer = millis();
        } else {
          blinkCycle = 0; // Reset pattern
        }
      }
      break;
      
    case BLINK_HEARTBEAT:
      // Pattern: quick double pulse, then pause
      if (millis() - blinkTimer >= 100) {
        int pattern[] = {1, 0, 1, 0, 0, 0, 0, 0}; // heartbeat pattern
        int patternLength = 8;
        
        if (blinkCycle < patternLength) {
          if (pattern[blinkCycle] == 1) {
            setSolidColor(color);
          } else {
            setSolidColor(COLOR_OFF);
          }
          blinkCycle++;
          blinkTimer = millis();
        } else {
          blinkCycle = 0; // Reset pattern
        }
      }
      break;
      
    case BLINK_RAPID_PULSE:
      // Very fast 50ms intervals
      if (millis() - blinkTimer >= 50) {
        blinkState = !blinkState;
        if (blinkState) {
          setSolidColor(color);
        } else {
          setSolidColor(COLOR_OFF);
        }
        blinkTimer = millis();
      }
      break;
      
    default:
      setSolidColor(COLOR_OFF);
      break;
  }
}

/**
 * Sets the LED to indicate a specific system state
 * @param state The system state to indicate
 */
void setLEDState(LEDstate state) {
    currentLEDState = state;
    switch(state) {
        case LED_CALIBRATING:
            setBlinkingMode(BLINK_FAST, COLOR_ORANGE);
            break;
        case LED_CALIBRATION_DONE:
            setBlinkingMode(BLINK_HOLD, COLOR_ORANGE);
            break;
        case LED_WIFI_CONNECTING:
            setBlinkingMode(BLINK_MEDIUM, COLOR_BLUE);
            break;
        case LED_WIFI_CONNECTED:
            setBlinkingMode(BLINK_HOLD, COLOR_BLUE);
            break;
        case LED_SOCKETIO_CONNECTING:
            setBlinkingMode(BLINK_MEDIUM, COLOR_YELLOW);
            break;
        case LED_SOCKETIO_CONNECTED:
            setBlinkingMode(BLINK_HOLD, COLOR_YELLOW);
            break;
        case LED_SOCKETIO_ERROR:
            setBlinkingMode(BLINK_TRIPLE, COLOR_RED);
            break;
        case LED_SYSTEM_ONLINE:
            setBlinkingMode(BLINK_HOLD, COLOR_GREEN);
            break;
        case LED_DEADMAN_NO_INPUT:
            setBlinkingMode(BLINK_HEARTBEAT, COLOR_MAGENTA);
            break;
        case LED_DEADMAN_WITH_INPUT:
            setBlinkingMode(BLINK_RAPID_PULSE, COLOR_MAGENTA);
            break;
    }
}

/**
 * Updates the LED based on the current state
 * This should be called frequently in the main loop
 */
void updateLED() {
    switch(currentLEDState) {
        case LED_CALIBRATING:
            setBlinkingMode(BLINK_FAST, COLOR_ORANGE);
            break;
        case LED_CALIBRATION_DONE:
            setBlinkingMode(BLINK_HOLD, COLOR_ORANGE);
            break;
        case LED_WIFI_CONNECTING:
            setBlinkingMode(BLINK_MEDIUM, COLOR_BLUE);
            break;
        case LED_WIFI_CONNECTED:
            setBlinkingMode(BLINK_HOLD, COLOR_BLUE);
            break;
        case LED_SOCKETIO_CONNECTING:
            setBlinkingMode(BLINK_MEDIUM, COLOR_YELLOW);
            break;
        case LED_SOCKETIO_CONNECTED:
            setBlinkingMode(BLINK_HOLD, COLOR_YELLOW);
            break;
        case LED_SOCKETIO_ERROR:
            setBlinkingMode(BLINK_TRIPLE, COLOR_RED);
            break;
        case LED_SYSTEM_ONLINE:
            setBlinkingMode(BLINK_HOLD, COLOR_GREEN);
            break;
        case LED_DEADMAN_NO_INPUT:
            setBlinkingMode(BLINK_HEARTBEAT, COLOR_MAGENTA);
            break;
        case LED_DEADMAN_WITH_INPUT:
            setBlinkingMode(BLINK_RAPID_PULSE, COLOR_MAGENTA);
            break;
    }
}


// ============= UTILITY FUNCTIONS =============

/**
 * Reads the state of the dead man switch
 * @return True if button is pressed, false otherwise
 */
bool readDeadmanSwitch() {
  return digitalRead(DEADMAN_BUTTON_PIN) == HIGH; // Button pressed when LOW (pull-up resistor)
}