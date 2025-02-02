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

#include <Wire.h>

// MPU9250 I2C address
#define MPU9250_ADDRESS 0x68

// Register addresses
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43
#define MAG_XOUT_L      0x03    // Magnetometer data starts at this register

#define MAX_DISTANCE 400
#define MIN_DISTANCE 10

// SLAM variables
#define MAX_LANDMARKS 100
float landmarks[MAX_LANDMARKS][2]; // Array to store landmark positions
int num_landmarks = 0;

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

float mapVibration(float distance){
  if (distance > 400) return -1;
  if (distance < 10) return 15;
  return pow(2.0,((float)map(distance,10,400,-1000,1000))/1000) * 50;
}

void vibrate(int delayAmt, int PIN_1, int PIN_2){
  return;
    if(delayAmt < 0) return;
    digitalWrite(PIN_2, HIGH);
    digitalWrite(PIN_1, LOW);

    // int centerFreq = map(centerDistance, 10, 400, 15, 50);

    delay(delayAmt);

    digitalWrite(PIN_1, HIGH);
    digitalWrite(PIN_2, LOW);

    delay(delayAmt);
}

float getDistance(int TRIG_PIN, int ECHO_PIN){
  // Send a 10-microsecond pulse to TRIG

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in centimeters
  float distance = duration * 0.034 / 2;

  // Serial.println(distance);
  // Serial.println(TRIG_PIN);
  // Serial.println(ECHO_PIN);
  return distance;

}

void setup() {

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

  Serial.begin(9600);       // Initialize Serial Monitor
  Serial.println("HC-SR04 Ultrasonic Sensor Initialized");

  setupWire();

  // Initialize landmarks array
  for (int i = 0; i < MAX_LANDMARKS; i++) {
    landmarks[i][0] = 0;
    landmarks[i][1] = 0;
  }

  // mpu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
}

int counter = 0;
bool suppress = false;
float prev_time = 0;

void loop() {

  float current_time = millis() / 1000.0;
  float dt = current_time - prev_time;

  uint64_t startTime = esp_timer_get_time();

  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar=='s') suppress = !suppress;
    if (incomingChar=='c') {
      num_landmarks = 0;
      robot_x = 0; robot_y = 0;
      robot_vx = 0; robot_vy = 0;
      robot_yaw = 0;
    }
  }

  if (suppress) counter = 1;
  counter = (counter+1)%10;

  int centerDistance = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  int leftDistance = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  int rightDistance = getDistance(TRIG_PIN_3, ECHO_PIN_3);

  int centerFreq = mapVibration(centerDistance);
  int leftFreq = mapVibration(leftDistance);
  int rightFreq = mapVibration(rightDistance);

  vibrate(leftFreq, MOTOR_1_PIN_1, MOTOR_1_PIN_2);
  vibrate(centerFreq, MOTOR_2_PIN_1, MOTOR_2_PIN_2);
  vibrate(rightFreq, MOTOR_3_PIN_1, MOTOR_3_PIN_2);

  // Read gyroscope data
  imnotevenfuckingpaidforthis();

  float delta_yaw = GZ; // Assuming 10ms loop time
  float accel_x = AX;
  float accel_y = AY;
  
  // Read ultrasound data
  unsigned int distance = centerDistance;
  center_dist = distance;
  
  // Update robot position
  updateRobotPosition(accel_x, accel_y, delta_yaw, dt);
  
  // SLAM process
  updateSLAM(distance);
  
  // Print current state
  if (counter==0) printState();

  delay(10); // 10ms delay
  uint64_t elapsedTime = esp_timer_get_time() - startTime;
  
  if (counter==0) {
    Serial.print("Elapsed: "); Serial.println(current_time - prev_time);
  }

  prev_time = current_time;
}

#define ACCEL_THRESHOLD 0.005
#define GYRO_THRESHOLD 0.005
#define ZUPT_DURATION 0.5 // s

unsigned long lastUpdateTime = 0;
bool isStationary = false;
unsigned long stationaryStartTime = 0;

void updateRobotPosition(float accel_x, float accel_y, float delta_yaw, float delta_t) {
  // Simple dead reckoning (this can be improved with more sophisticated methods)
  
  float accelMagnitude = sqrt(sq(AX) + sq(AY) + sq(AZ-1)); // / 16384.0; // Convert to g
  float gyroMagnitude = sqrt(sq(GX) + sq(GY) + sq(GZ)); // / 131.0; // Convert to deg/s

  // Update position (double integration of acceleration)

  if (accelMagnitude < ACCEL_THRESHOLD && gyroMagnitude < GYRO_THRESHOLD) {
    if (!isStationary) {
      isStationary = true;
      stationaryStartTime = prev_time;
    }
    
    if (prev_time - stationaryStartTime >= ZUPT_DURATION) {
      // Apply ZUPT correction
      robot_vx = 0; robot_vy = 0;
      if (counter==0) Serial.println("ZUPT applied");
    }
  } else {
    isStationary = false;
    
    if (accelMagnitude < ACCEL_THRESHOLD){
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

void updateSLAM(unsigned int distance) {
  if (distance > MIN_DISTANCE && distance < MAX_DISTANCE) {
    // Convert polar coordinates to Cartesian, relative to robot
    float relative_x = distance * cos(robot_yaw);
    float relative_y = distance * sin(robot_yaw);
    
    // Convert to global coordinates
    float global_x = robot_x + relative_x;
    float global_y = robot_y + relative_y;
    
    // Check if this is a new landmark
    bool is_new_landmark = true;
    for (int i = 0; i < num_landmarks; i++) {
      float dx = global_x - landmarks[i][0];
      float dy = global_y - landmarks[i][1];
      float distance_to_landmark = sqrt(dx*dx + dy*dy);
      
      if (distance_to_landmark < 1) { // Threshold for considering it the same landmark
        is_new_landmark = false;
        // Update landmark position (simple average)
        landmarks[i][0] = (landmarks[i][0] + global_x) / 2;
        landmarks[i][1] = (landmarks[i][1] + global_y) / 2;
        break;
      }
    }
    
    // Add new landmark if it's new and we haven't reached the maximum
    if (is_new_landmark && num_landmarks < MAX_LANDMARKS) {
      landmarks[num_landmarks][0] = global_x;
      landmarks[num_landmarks][1] = global_y;
      num_landmarks++;
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
  Serial.println(num_landmarks);
  
  Serial.print("Landmarks: \n[");
  for (int i = 0; i < num_landmarks; i++) {
    // Serial.print("Landmark ");
    // Serial.print(i);
    Serial.print("(");
    Serial.print(landmarks[i][0]);
    Serial.print(", ");
    Serial.print(landmarks[i][1]);
    Serial.print("), ");
  }
  Serial.println("(0,0)]");
}

void setupWire() {
  Wire.begin();
  Serial.begin(115200);
  
  // Wake up MPU9250
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // set to zero (wakes up the MPU-9250)
  Wire.endTransmission(true);

  // Configure accelerometer range (±2g)
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // set to ±2g range
  Wire.endTransmission(true);

  // Configure gyroscope range (±250 deg/s)
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // set to ±250 deg/s range
  Wire.endTransmission(true);
}

void imnotevenfuckingpaidforthis() {
  // Read accelerometer data
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6, true);
  
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();
  
  // Convert raw values to g force (±2g range)
  float ax = accelX / 16384.0;
  float ay = accelY / 16384.0;
  float az = accelZ / 16384.0;
  AX = ax; AY = ay; AZ = az;

  // Read gyroscope data
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6, true);
  
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert raw values to degrees per second (±250 deg/s range)
  float gx = gyroX / 131.0 * PI / 180;
  float gy = gyroY / 131.0 * PI / 180;
  float gz = gyroZ / 131.0 * PI / 180;
  GX = gx; GY = gy; GZ = gz;

  if(counter!=0) return;

  // Print values

  Serial.print("Position: ");
  Serial.print(robot_x); Serial.print(", ");
  Serial.print(robot_y); Serial.print(" | ");

  Serial.print("Velocity: ");
  Serial.print(robot_vx); Serial.print(", ");
  Serial.print(robot_vy); Serial.print(" | ");

  Serial.print("Accel (g): ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az); Serial.print(" | ");
  
  Serial.print("Gyro (rad/s): ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.println(gz);
}
