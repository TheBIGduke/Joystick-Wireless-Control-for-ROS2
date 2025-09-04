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
 *
 * @author Kaléin Tamaríz - TheBIGduke
 * @version 1.0.0
 *
 */
#include <Arduino.h>            // Arduino core functions for PlatformIO
#include <WiFi.h>               // ESP32 WiFi functionality
#include <SocketIOclient.h>     // Socket.IO client library

// ============= CONFIGURATION SECTION =============
// Update these values for your specific setup

// WiFi credentials
const char* ssid = "ESP32";
const char* password = "12345678";

// Socket.IO server (replace with your computer's IP where ROS2 robot is running)
const char* socketio_host = "192.168.0.103";
const int socketio_port = 9009;

// Joystick pins
#define VRX_PIN  39 // ESP32 pin GPIO39 (ADC3) connected to VRX pin
#define VRY_PIN  36 // ESP32 pin GPIO36 (ADC0) connected to VRY pin

// Joystick calibration values - will be automatically calibrated on startup
int CENTER_X_CALIBRATED = 2048; // Default center, will be updated during calibration
int CENTER_Y_CALIBRATED = 2048; // Default center, will be updated during calibration
#define MAX_X    4095
#define MAX_Y    4095

// Movement parameters
const float LINEAR_SPEED_FULL = 0.4;   // Full linear velocity (m/s)
const float LINEAR_SPEED_HALF = 0.2;   // Half linear velocity (m/s)
const float ANGULAR_SPEED_FULL = 0.6;  // Full angular velocity (rad/s)
const float ANGULAR_SPEED_HALF = 0.3;  // Half angular velocity (rad/s)

// Speed thresholds (based on joystick range)
const int SPEED_THRESHOLD = 250;  // Threshold for half vs full speed

// Robot states
enum RobotState {
  FORWARD,
  ROTATE_RIGHT,
  ROTATE_LEFT,
  BACKWARD
};

SocketIOclient socketIO;

// Joystick and control variables
int valueX = 0; // to store the X-axis value
int valueY = 0; // to store the Y-axis value
unsigned long lastCmdVel = 0;
const unsigned long CMD_VEL_INTERVAL = 100; // Interval to send cmd_vel commands

// Joystick calibration functionc
void calibrateJoystick() {
  Serial.println("=== JOYSTICK CALIBRATION ===");
  Serial.println("Please center your joystick and keep it still...");
  Serial.println("Calibrating in 5 seconds...");
  
  // Countdown
  for(int i = 5; i > 0; i--) {
    Serial.println(String(i) + "...");
    delay(1000);
  }
  
  Serial.println("Calibrating... DO NOT MOVE THE JOYSTICK!");
  
  // Take many readings and average them for better accuracy
  long sumX = 0, sumY = 0;
  int samples = 1000;
  
  for(int i = 0; i < samples; i++) {
    sumX += analogRead(VRY_PIN);
    sumY += analogRead(VRX_PIN);
    delay(20);
    if(i % 10 == 0) Serial.print(".");
  }
  Serial.println();
  
  CENTER_X_CALIBRATED = sumX / samples;
  CENTER_Y_CALIBRATED = sumY / samples;
  
  Serial.println("Calibration complete!");
  Serial.println("CENTER_X: " + String(CENTER_X_CALIBRATED));
  Serial.println("CENTER_Y: " + String(CENTER_Y_CALIBRATED));
  
  // Test the calibration
  Serial.println("\nTesting calibration (keep joystick centered):");
  for(int i = 0; i < 10; i++) {
    int testX = analogRead(VRY_PIN);
    int testY = analogRead(VRX_PIN);
    int mappedX = (testX - CENTER_X_CALIBRATED) * 1000 / MAX_X;
    int mappedY = (testY - CENTER_Y_CALIBRATED) * 1000 / MAX_Y;
    Serial.println("Raw(" + String(testX) + "," + String(testY) + ") -> Mapped(" + String(mappedX) + "," + String(mappedY) + ")");
    delay(200);
  }
  
  Serial.println("\n=== CALIBRATION VALUES ===");
  Serial.println("CENTER_X: " + String(CENTER_X_CALIBRATED));
  Serial.println("CENTER_Y: " + String(CENTER_Y_CALIBRATED));
  Serial.println("============================\n");
  
  delay(2000);
}

// Socket.IO event handler
void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case sIOtype_DISCONNECT:
      Serial.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      Serial.printf("[IOc] Connected to url: %s\n", payload);
      // Join default namespace (no auto join in Socket.IO)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      Serial.printf("[IOc] get event: %s\n", payload);
      break;
    case sIOtype_ACK:
      Serial.printf("[IOc] get ack: %u\n", length);
      break;
    case sIOtype_ERROR:
      Serial.printf("[IOc] get error: %u\n", length);
      break;
    case sIOtype_BINARY_EVENT:
    case sIOtype_BINARY_ACK:
      // Not used in this example
      break;
  }
}

String getMovementDescription(float linear_x, float angular_z) {
  if (linear_x > 0 && angular_z == 0) return "FORWARD";
  else if (linear_x < 0 && angular_z == 0) return "BACKWARD";
  else if (linear_x == 0 && angular_z > 0) return "ROTATE_LEFT";
  else if (linear_x == 0 && angular_z < 0) return "ROTATE_RIGHT";
  else if (linear_x == 0 && angular_z == 0) return "STOPPED";
  else return "COMBINED_MOVEMENT";
}

void sendCmdVel(float linear_x, float angular_z) {
  // Create cmd_vel JSON payload
  String payload = "[\"cmd_vel\",{\"linear_x\":" + String(linear_x) + ",\"angular_z\":" + String(angular_z) + "}]";
  
  // Send cmd_vel event to ROS2 robot
  socketIO.sendEVENT(payload);
  
  Serial.println("Movement: " + getMovementDescription(linear_x, angular_z) + " | linear_x=" + String(linear_x) + ", angular_z=" + String(angular_z));
}

void readJoystick() {
  // Read X and Y analog values
  valueX = analogRead(VRY_PIN);
  valueY = analogRead(VRX_PIN);

  // Map so center is 0, left/up is -100 and right/down is +100
  // Use calibrated center values
  valueX = (valueX - CENTER_X_CALIBRATED) * 1000 / MAX_X;
  valueY = (valueY - CENTER_Y_CALIBRATED) * 1000 / MAX_Y;

  // Apply custom deadzone
  const int deadzone = 10; // Adjust as needed
  if (abs(valueX) < deadzone) valueX = 0;
  if (abs(valueY) < deadzone) valueY = 0;

  // Debug output
  Serial.print("Joystick - x = ");
  Serial.print(valueX);
  Serial.print(", y = ");
  Serial.println(valueY);
}

float calculateLinearSpeed(int joystickValue) {
  int absValue = abs(joystickValue);
  
  if (absValue == 0) {
    return 0.0;
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

float calculateAngularSpeed(int joystickValue) {
  int absValue = abs(joystickValue);
  
  if (absValue == 0) {
    return 0.0;
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

void processJoystickInput() {
  float linear_x = 0.0;
  float angular_z = 0.0;
  
  // Calculate speeds based on joystick positions
  // Forward/Backward control (Y-axis)
  if (valueY != 0) {
    linear_x = calculateLinearSpeed(valueY);
  }
  
  // Left/Right rotation control (X-axis)
  if (valueX != 0) {
    angular_z = -calculateAngularSpeed(valueX); // Negative for correct rotation direction
  }
  
  sendCmdVel(linear_x, angular_z);
}

void setup() {
  Serial.begin(9600);
  Serial.setDebugOutput(true);

  // Set the ADC attenuation to 11 dB (up to ~3.3V input)
  analogSetAttenuation(ADC_11db);
  
  delay(2000); // Give time to open Serial Monitor
  
  // Run automatic joystick calibration
  calibrateJoystick();
 
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  
  // Verify connection and return state
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Setup Socket.IO client
  socketIO.begin(socketio_host, socketio_port, "/socket.io/?EIO=4");
  socketIO.onEvent(socketIOEvent);
  
  Serial.println("=== Joystick Wireless Control for ROS2 Started ===");
  Serial.println("Joystick Controls with Variable Speed:");
  Serial.println("- Y-axis: Forward/Backward movement");
  Serial.println("- X-axis: Left/Right rotation");
  Serial.println("- Speed: Half speed for |value| <= " + String(SPEED_THRESHOLD) + ", Full speed for |value| > " + String(SPEED_THRESHOLD));
  Serial.println("- Linear speeds: Half=" + String(LINEAR_SPEED_HALF) + "m/s, Full=" + String(LINEAR_SPEED_FULL) + "m/s");
  Serial.println("- Angular speeds: Half=" + String(ANGULAR_SPEED_HALF) + "rad/s, Full=" + String(ANGULAR_SPEED_FULL) + "rad/s");
  Serial.println("\nNow testing with live joystick movement...");
  Serial.println("Move your joystick around to test the calibration:");
}

void loop() {
  socketIO.loop();
  
  // Read joystick input
  readJoystick();
  
  // Send cmd_vel commands at regular intervals
  if (millis() - lastCmdVel >= CMD_VEL_INTERVAL) {
    lastCmdVel = millis();
    processJoystickInput();
  }
 
  delay(200); // Delay to match your joystick reading rate
}