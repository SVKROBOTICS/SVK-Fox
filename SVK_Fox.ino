#include <SVKIR8.h>

#define MAX_INTEGRAL 1400 // Maximun value of integral variable
#define MAX_SPEED 160 // Maximum allowed speed

// Create class object
SVKIR8 irSensors;

// Sets sensor amount and pins
const uint8_t sensorCount = 8;
const uint8_t muxPins[4] = { 10, 9, 8, A0};

// Creates array to store Ir sensor values
uint16_t sensorValues[sensorCount];

// PID constants
float Kp = 0.03;      // Proportional constant
float Ki = 0.0001;    // Integral constant
float Kd = 0.5;     // Derivative constant

// Motor Pins
const uint8_t MOTOR1_IN1 = 2;
const uint8_t MOTOR1_IN2 = 3;
const uint8_t MOTOR1_PWM = 5;
const uint8_t MOTOR2_IN1 = 4;
const uint8_t MOTOR2_IN2 = 7;
const uint8_t MOTOR2_PWM = 6;

// Ultrasonic Pins
const int trigPin = A3;
const int echoPin = A1;

float distance;

// PID variables
float lastError = 0;
float integral = 0;

// Motor Speed variables
const int baseSpeed = 65;
int leftSpeed = 0;
int rightSpeed = 0;


float getDistanceCM() {
  // Send a 10µs HIGH pulse to trigger the ultrasonic burst
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse, with 5ms timeout
  long duration = pulseIn(echoPin, HIGH, 5000);

  if (duration == 0) {
    // Timeout occurred, no pulse received
    return -1; // or some invalid distance to indicate no reading
  }

  // Calculate distance in cm (speed of sound is approx. 343 m/s)
  float distance = duration * 0.0343 / 2;

  return distance;
}


void setup() {
    irSensors.setMultiplexerPins(muxPins);

    // Motor pin assignments
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);

    // Set both motor direction FORWARD
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);

    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR2_PWM, 0);

    // Sets samples taken in each loop for each sensor
    irSensors.setSamplesPerSensor(2);

    // Runs the calibrate 300 times for the robot to get max and min values read
    for (uint16_t i = 0; i < 300; i++) {
        irSensors.calibrate();
    }

    Serial.begin(9600);

    // Prints minimum and maximum values read by irSensors
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(irSensors._calibration.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(irSensors._calibration.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();


    // Ultrasonic pins assignments
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Adds delay to be able to place in starting position
    delay(1500);
}

void loop() {

  distance = getDistanceCM();

  if(distance > 0 && distance <= 20) {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, HIGH);
    digitalWrite(MOTOR2_IN1, HIGH);
    digitalWrite(MOTOR2_IN2, HIGH);
    Serial.println("Object detected ahead, stopping...");
  } else {
    digitalWrite(MOTOR1_IN1, HIGH);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, HIGH);
    // read calibrated irSensors values and get position of black line from 0 to 7000 (8 irSensors)
    float position = irSensors.readLineBlack();
    float error = 3500 - position; // Assuming the line is at the middle (3500)

    integral += error;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float derivative = error - lastError;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    // Adjust motor speeds based on PID output
    leftSpeed = baseSpeed - output;
    rightSpeed = baseSpeed + output;

    // Ensure motor speeds don't exceed maximum speed limit
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

    // Control the motors
    analogWrite(MOTOR2_PWM, leftSpeed); // Left motor speed control
    analogWrite(MOTOR1_PWM, rightSpeed); // Right motor speed control

    // Add a small delay to allow motors to adjust
    delayMicroseconds(1000);
  }
}
