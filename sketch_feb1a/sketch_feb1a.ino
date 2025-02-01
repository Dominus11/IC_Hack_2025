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

#define MAX_DIST = 400
#define MIN_DIST = 10


void vibrate(int delayAmt, int PIN_1, int PIN_2){
    digitalWrite(PIN_2, HIGH);
    digitalWrite(PIN_1, LOW);

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

  Serial.println(distance);
  Serial.println(TRIG_PIN);
  Serial.println(ECHO_PIN);
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
}

void loop() {
  int centerDistance = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  int leftDistance = getDistance(TRIG_PIN_2, ECHO_PIN_2);
  int rightDistance = getDistance(TRIG_PIN_3, ECHO_PIN_3);

  int centerFreq = map(centerDistance, 10, 400, 15, 50);
  int leftFreq = map(leftDistance, 10, 400, 15, 50);
  int rightFreq = map(rightDistance, 10, 400, 15, 50);

  vibrate(leftFreq, MOTOR_1_PIN_1, MOTOR_1_PIN_2);
  vibrate(centerFreq, MOTOR_2_PIN_1, MOTOR_2_PIN_2);
  vibrate(rightFreq, MOTOR_3_PIN_1, MOTOR_3_PIN_2);
}
