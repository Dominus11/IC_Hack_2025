long duration;
int distance;

const int ECHO_PIN_1 = 18; 
const int TRIG_PIN_1 = 19;
const int MOTOR_PIN_1 = 4;
const int MOTOR_PIN_2 = 5;
int delayAmt = 50;

void setup() {
  pinMode(TRIG_PIN_1, OUTPUT);  // Set TRIG as output
  pinMode(ECHO_PIN_1, INPUT);   // Set ECHO as input
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);

  Serial.begin(9600);       // Initialize Serial Monitor
  Serial.println("HC-SR04 Ultrasonic Sensor Initialized");

}

void loop() {
  // Send a 10-microsecond pulse to TRIG
  /*
  digitalWrite(TRIG_PIN_1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_1, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN_1, HIGH);

  // Calculate distance in centimeters
  float distance = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500);  // Wait for half a second
*/
  int i = 0 ;


  while (i * delayAmt < 500){
    digitalWrite(MOTOR_PIN_2, HIGH);
    digitalWrite(MOTOR_PIN_1, LOW);

    delay(delayAmt);

    digitalWrite(MOTOR_PIN_1, HIGH);
    digitalWrite(MOTOR_PIN_2, LOW);

    delay(delayAmt);
    i++;
  }

  delayAmt--;
  Serial.println(delayAmt);

}
