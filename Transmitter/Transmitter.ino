const int ECHO_PIN_1 = 16; // FRONT CAMERA 
const int TRIG_PIN_1 = 17; // FRONT CAMERA
const int ECHO_PIN_2 = 19; // LEFT CAMERA
const int TRIG_PIN_2 = 18; // LEFT CAMERA
const int ECHO_PIN_3 = 4; // RIGHT CAMERA
const int TRIG_PIN_3 = 5; // RIGHT CAMERA
const int MOTOR_1_PIN_1 = 26; // LEFT MOTOR 
const int MOTOR_1_PIN_2 = 27; // LEFT MOTOR
const int MOTOR_2_PIN_1 = 12; // CENTER MOTOR 
const int MOTOR_2_PIN_2 = 13; // CENTER MOTOR
const int MOTOR_3_PIN_1 = 32; // RIGHT MOTOR 
const int MOTOR_3_PIN_2 = 33; // RIGHT MOTOR
const int SDA_PIN = 21; // 
const int SCL_PIN = 22; //

const int BUZZER = 15;
int buzz = 0;

#include "FastIMU.h"
#include <Wire.h>
#include <MadgwickAHRS.h> // Include the Madgwick filter library

#include <WiFi.h>
#include <ArduinoWebsockets.h>

const char* ssid = "LAPTOP-Q2S8O6B7 4664";
const char* password = "1[Y662e3";
const char* server_url = "ws://192.168.137.64:8765";  // Replace with laptop's IP

using namespace websockets;
WebsocketsClient client;

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6500 IMU;               //Change to the name of any supported IMU! 

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

// Initialize Madgwick filter
Madgwick filter;
const float sensorRate = 104.00; // IMU sample rate in Hz

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

#define MAX_DISTANCE 400
#define MIN_DISTANCE 10

// SLAM variables
#define MAX_LANDMARKS 50
float landmarks[MAX_LANDMARKS][2]; // Array to store landmark positions
int num_landmarks = 0;
int overflow = false;

float dists[3];
unsigned long lastResetTime = 0;

// Robot position and movement
float robot_x = 0;
float robot_y = 0;
float robot_vx = 0;
float robot_vy = 0;
float robot_yaw = 0;
float center_dist = 0;

// double delta_t = 0;

float AX = 0;
float AY = 0;
float AZ = 0;
float GX = 0;
float GY = 0;
float GZ = 0;

void setupWifi(){
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
        // break;
    }
    Serial.println("Connected to WiFi");

    // Set up WebSocket event handlers
    client.onMessage([](WebsocketsMessage message) {
        Serial.print("Received: ");
        Serial.println(message.data());
        buzz = message.data().toInt() * 100;
    });

    client.onEvent([](WebsocketsEvent event, String data) {
        if (event == WebsocketsEvent::ConnectionOpened) {
            Serial.println("WebSocket connection opened");
        } else if (event == WebsocketsEvent::ConnectionClosed) {
            Serial.println("WebSocket connection closed");
        } else if (event == WebsocketsEvent::GotPing) {
            Serial.println("Received a ping");
        } else if (event == WebsocketsEvent::GotPong) {
            Serial.println("Received a pong");
        }
    });

    // Connect to WebSocket server
    bool connected = client.connect(server_url);
    if (connected) {
        Serial.println("Connected to WebSocket server");
    } else {
        Serial.println("WebSocket connection failed");
    }
}

float mapVibration(float distance){
  if (distance < 5 || distance > 400) return 0;
  return 1/map(distance, 5, 400, 1, 1000);
}

#define CYCLE 100

bool direction = false;
void updateVibration(unsigned long now){

  if (now - lastResetTime > CYCLE){
    digitalWrite(MOTOR_1_PIN_1, LOW);
    digitalWrite(MOTOR_1_PIN_2, LOW);

    digitalWrite(MOTOR_2_PIN_1, LOW);
    digitalWrite(MOTOR_2_PIN_2, LOW);

    digitalWrite(MOTOR_3_PIN_1, LOW);
    digitalWrite(MOTOR_3_PIN_2, LOW);

    lastResetTime = now;
  }

  if (now - lastResetTime > CYCLE * mapVibration(dists[0])) {
    digitalWrite(MOTOR_1_PIN_2, direction ? HIGH : LOW);
    digitalWrite(MOTOR_1_PIN_1, direction ? LOW : HIGH);
  }
  if (now - lastResetTime > CYCLE * mapVibration(dists[1])) {
    digitalWrite(MOTOR_2_PIN_2, direction ? HIGH : LOW);
    digitalWrite(MOTOR_2_PIN_1, direction ? LOW : HIGH);
  }
  if (now - lastResetTime > CYCLE * mapVibration(dists[2])) {
    digitalWrite(MOTOR_3_PIN_2, direction ? HIGH : LOW);
    digitalWrite(MOTOR_3_PIN_1, direction ? LOW : HIGH);
  }

  direction = !direction;

}

float getDistance(int TRIG_PIN, int ECHO_PIN){
  // Send a 10-microsecond pulse to TRIG

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // Calculate distance in centimeters
  float distance = duration * 0.034 / 2;

  // Serial.println(distance);
  // Serial.println(TRIG_PIN);
  // Serial.println(ECHO_PIN);
  return distance;

}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  setupWifi();

  pinMode(TRIG_PIN_1, OUTPUT);  // Set TRIG as output
  pinMode(ECHO_PIN_1, INPUT);   // Set ECHO as input
  pinMode(TRIG_PIN_2, OUTPUT);  // Set TRIG as output
  pinMode(ECHO_PIN_2, INPUT);   // Set ECHO as input
  pinMode(TRIG_PIN_3, OUTPUT);  // Set TRIG as output
  pinMode(ECHO_PIN_3, INPUT);   // Set ECHO as input
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);  
  pinMode(MOTOR_2_PIN_1, OUTPUT);
  pinMode(MOTOR_2_PIN_2, OUTPUT);
  pinMode(MOTOR_3_PIN_1, OUTPUT);
  pinMode(MOTOR_3_PIN_2, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  delay(1000);
  IMU.init(calib, IMU_ADDRESS);

  filter.begin(sensorRate);

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  // Initialize landmarks array
  for (int i = 0; i < MAX_LANDMARKS; i++) {
    landmarks[i][0] = 0;
    landmarks[i][1] = 0;
  }

  for (int i = 0; i < 3; i++) {
    dists[i] = -1;
  }

  // mpu.setAccelRange(MPU6500::ACCEL_RANGE_8G);
}

int counter = 0;
bool suppress = false;
float prev_time = 0;

String buffer = "";

int centerDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

void loop() {

  float current_time = millis() / 1000.0;
  float dt = current_time - prev_time;

  uint64_t startTime = esp_timer_get_time();

  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar=='s') suppress = !suppress;
    if (incomingChar=='c') {
      num_landmarks = 0;
      overflow = false;
      robot_x = 0; robot_y = 0;
      robot_vx = 0; robot_vy = 0;
      robot_yaw = 0;
    }
  }

  if (suppress) counter = 1;
  counter = (counter+1)%10;

  centerDistance = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  leftDistance = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  rightDistance = getDistance(TRIG_PIN_3, ECHO_PIN_3);

  // Read gyroscope data
  imnotevenfuckingpaidforthis();

  float delta_yaw = GZ; // Assuming 10ms loop time
  float accel_x = AX;
  float accel_y = AY;
  
  // Read ultrasound data
  center_dist = centerDistance;
  
  // Update robot position
  updateRobotPosition(accel_x, accel_y, delta_yaw, dt);
  
  // SLAM process
  updateSLAM(centerDistance, 0);
  updateSLAM(leftDistance, -PI/6);
  updateSLAM(rightDistance, PI/6);

  updateDists();
  updateVibration(millis());
  
  // Print current state
  if (counter==0) printState();

  if (client.available()) {
    client.poll();  // Keep connection alive and handle incoming messages
  }
  tone(BUZZER, buzz);

  // Optional: Send a message to the server periodically
  unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 5000) {  // Send every 5 seconds
      client.send(buffer);
      buffer = "";
      lastSendTime = millis();
  }

  uint64_t elapsedTime = esp_timer_get_time() - startTime;
  
  if (counter==0) {
    Serial.print("Elapsed: "); Serial.println(current_time - prev_time);
  }

  prev_time = current_time;

}

#define ACCEL_THRESHOLD 0.2
#define GYRO_THRESHOLD 0.01
#define ZUPT_DURATION 0.5 // s

unsigned long lastUpdateTime = 0;
bool isStationary = false;
unsigned long stationaryStartTime = 0;

float IX = 0;
float IY = 0;
float IZ = 9.8;

void updateRobotPosition(float accel_x, float accel_y, float delta_yaw, float delta_t) {
  // Simple dead reckoning (this can be improved with more sophisticated methods)
  
  float accelMagnitude = sqrt(sq(AX) + sq(AY) + sq(AZ)) - 9.81; // / 16384.0; // Convert to g
  float gyroMagnitude = sqrt(sq(GX) + sq(GY) + sq(GZ)); // / 131.0; // Convert to deg/s

  // Update position (double integration of acceleration)

  if (accelMagnitude < ACCEL_THRESHOLD && gyroMagnitude < GYRO_THRESHOLD) {
    if (!isStationary) {
      isStationary = true;
      stationaryStartTime = prev_time;
    }
    
    if (counter==0) Serial.println("ZUPT suspected");
    
    if (prev_time - stationaryStartTime >= ZUPT_DURATION) {
      // Apply ZUPT correction
      robot_vx = 0; robot_vy = 0;
      if (counter==0) Serial.println("ZUPT applied");
    }
  } else {
    isStationary = false;
    
    if (accelMagnitude >= ACCEL_THRESHOLD){
      // Update velocity and position
      robot_vx += accel_x * delta_t;
      robot_vy += accel_y * delta_t;

      robot_x += robot_vx * delta_t;
      robot_y += robot_vy * delta_t;
    }

    // Update yaw
    if (gyroMagnitude >= GYRO_THRESHOLD) robot_yaw += delta_yaw * delta_t;
  }
  
  // Normalize yaw to [-π, π]
  while (robot_yaw > PI) robot_yaw -= 2 * PI;
  while (robot_yaw < -PI) robot_yaw += 2 * PI;
}

void updateSLAM(unsigned int distance, int theta) {
  if (distance > MIN_DISTANCE && distance < MAX_DISTANCE) {
    // Convert polar coordinates to Cartesian, relative to robot
    float relative_x = distance * cos(robot_yaw+theta) / 100;
    float relative_y = distance * sin(robot_yaw+theta) / 100;
    
    // Convert to global coordinates
    float global_x = robot_x + relative_x;
    float global_y = robot_y + relative_y;
    
    // Check if this is a new landmark
    bool is_new_landmark = true;
    for (int i = overflow ? (num_landmarks+1)%MAX_LANDMARKS : 0; i != num_landmarks; i = (i+1)%MAX_LANDMARKS) {
      float dx = global_x - landmarks[i][0];
      float dy = global_y - landmarks[i][1];
      float distance_to_landmark = sqrt(dx*dx + dy*dy);
      
      if (distance_to_landmark < 1) { // Threshold for considering it the same landmark
        is_new_landmark = false;
        // Update landmark position (simple average)
        landmarks[i][0] = (landmarks[i][0] + 100 * global_x) / 2;
        landmarks[i][1] = (landmarks[i][1] + 100 * global_y) / 2;
        break;
      }
    }
    
    // Add new landmark if it's new and we haven't reached the maximum
    if (is_new_landmark) {
      landmarks[num_landmarks][0] = global_x;
      landmarks[num_landmarks][1] = global_y;
      num_landmarks = (num_landmarks+1)%MAX_LANDMARKS;
      if (num_landmarks == 0) overflow = true;
    }
  }
}

void printState() {
  Serial.println("Current State:");
  Serial.print("Yaw: ");
  Serial.print(robot_yaw);
  Serial.println(" radians");
  Serial.print("Distance: ");
  Serial.print(center_dist);
  Serial.println(" cm");
  Serial.print("Number of landmarks: ");
  Serial.println(overflow ? MAX_LANDMARKS : num_landmarks);
  
  Serial.print("Landmarks: \n[");
  buffer = String(100*robot_x) + " " + String(100*robot_y) + "; ";
  for (int i = overflow ? (num_landmarks+1)%MAX_LANDMARKS : 0; i != num_landmarks; i = (i+1)%MAX_LANDMARKS) {
    // Serial.print("Landmark ");
    // Serial.print(i);
    buffer += String(landmarks[i][0]) + " " + String(landmarks[i][1]) + "; ";
    Serial.print("(");
    Serial.print(landmarks[i][0]);
    Serial.print(", ");
    Serial.print(landmarks[i][1]);
    Serial.print("), ");
  }
  Serial.println("(0,0)]");
}

void imnotevenfuckingpaidforthis() {

  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  
  float ax = accelData.accelX;
  float ay = accelData.accelY;
  float az = accelData.accelZ;

  // Read gyroscope data
  GX = gyroData.gyroX * PI/180; 
  GY = gyroData.gyroY * PI/180; 
  GZ = gyroData.gyroZ * PI/180; 

  filter.updateIMU(GX, GY, GZ, ax, ay, az);
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  AX = ax - sin(pitch) * 9.81;
  AY = ay - sin(roll) * cos(pitch) * 9.81;
  AZ = az - cos(roll) * cos(pitch) * 9.81;

  if(counter!=0) return;

  // Print values

  Serial.print("Position: ");
  Serial.print(robot_x); Serial.print(", ");
  Serial.print(robot_y); Serial.print(" | ");

  Serial.print("Velocity: ");
  Serial.print(robot_vx); Serial.print(", ");
  Serial.print(robot_vy); Serial.print(" | ");

  Serial.print("Accel (g): ");
  Serial.print(AX); Serial.print(", ");
  Serial.print(AY); Serial.print(", ");
  Serial.print(AZ); Serial.print(" | ");
  
  Serial.print("Gyro (rad/s): ");
  Serial.print(GX); Serial.print(", ");
  Serial.print(GY); Serial.print(", ");
  Serial.println(GZ); Serial.print(" | ");

  Serial.print("Dists (m): ");
  Serial.print(dists[0]); Serial.print(", ");
  Serial.print(dists[1]); Serial.print(", ");
  Serial.println(dists[2]);
}

void updateDists(){
  // for(int j = 0; j < 3; j++){
  //   dists[j] = -1;
  //   float theta = (j-1)*PI/2;
  //   float tx = cos(theta);
  //   float ty = sin(theta);
  //   for (int i = 0; i < num_landmarks; i++){
  //     float x = landmarks[i][0] - robot_x;
  //     float y = landmarks[i][1] - robot_y;
  //     float d = sqrt(x*x+y*y);
  //     if (abs((tx*x+ty*y) / (x*x+y*y+0.001)) <= cos(PI/8) && (dists[j]>d || dists[j]<0)) {
  //       dists[j] = d;
  //     }
  //   }
  // }
  dists[0] = leftDistance;
  dists[1] = centerDistance;
  dists[2] = rightDistance;
}






