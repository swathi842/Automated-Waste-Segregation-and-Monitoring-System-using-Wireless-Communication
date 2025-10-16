#include <Servo.h>

// --- Pins ---
// Ultrasonic sensor
const int trigPin = 12;
const int echoPin = 13;

// Conveyor motor control (via relay or motor driver)
const int conveyorPin = 2;  // Digital pin to control conveyor ON/OFF

// Servos
Servo colorServo;   // SG90 servo for color sorting
Servo capServo;     // SG90 servo for plastic detection

// Color sensor pins (TCS3200)
#define S0 6
#define S1 7
#define S2 8
#define S3 9
#define colorOut 10

// Capacitive proximity sensor pin
const int capSensorPin = 4;

// Servo signal pins
const int colorServoPin = 3;
const int capServoPin = 5;

// Distance threshold for object detection (in cm)
const int distanceThreshold = 15;

void setup() {
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Conveyor control pin
  pinMode(conveyorPin, OUTPUT);
  digitalWrite(conveyorPin, HIGH); // Conveyor off initially

  // Capacitive sensor pin
  pinMode(capSensorPin, INPUT);

  // Color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);

  // Set TCS3200 frequency scaling to 100%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Attach servos
  colorServo.attach(colorServoPin);
  capServo.attach(capServoPin);

  // Initialize servos to neutral (90 degrees)
  colorServo.write(90);
  capServo.write(90);

  Serial.println("System Initialized");
}

void loop() {
  long distance = readUltrasonicDistance();

  if (distance > 0 && distance <= distanceThreshold) {
    // Object detected by ultrasonic sensor
    digitalWrite(conveyorPin, LOW);  // Turn ON conveyor (relay LOW = ON)
    Serial.println("Conveyor ON - Object detected");

    // Keep conveyor running for 4 seconds
    delay(30000);
    digitalWrite(conveyorPin, HIGH); // Turn OFF conveyor
    Serial.println("Conveyor OFF - Time elapsed");

    // Check capacitive sensor for plastic
    bool plasticDetected = digitalRead(capSensorPin) == HIGH;

    // Read color sensor frequencies
    int redFreq = readColorFrequency(LOW, LOW);
    int blueFreq = readColorFrequency(LOW, HIGH);
    int greenFreq = readColorFrequency(HIGH, HIGH);

    String detectedColor = detectColor(redFreq, greenFreq, blueFreq);

    // Activate servos based on sensors
    if (plasticDetected) {
      Serial.println("Plastic detected - Activating plastic servo");
      capServo.write(0);   // Move servo to action position
      delay(800);
      capServo.write(90);  // Return to neutral
    }

    if (detectedColor != "Unknown") {
      Serial.print("Color detected: ");
      Serial.println(detectedColor);

      // Move color servo based on color
      if (detectedColor == "Red") {
        colorServo.write(0);
      } else if (detectedColor == "Blue") {
        colorServo.write(45);
      } else if (detectedColor == "Green") {
        colorServo.write(135);
      }
      delay(1000);
      colorServo.write(90);  // Neutral
    }

  } else {
    // No object detected, conveyor OFF
    digitalWrite(conveyorPin, HIGH);
    Serial.println("Conveyor OFF - No object");
  }

  delay(50); // Short delay between loops
}

// Reads distance from ultrasonic sensor (HC-SR04)
long readUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms

  if (duration == 0) return -1; // No pulse received

  long distanceCm = duration * 0.034 / 2;
  return distanceCm;
}

// Reads TCS3200 color sensor frequency
int readColorFrequency(bool s2State, bool s3State) {
  digitalWrite(S2, s2State);
  digitalWrite(S3, s3State);
  delay(50);
  int freq = pulseIn(colorOut, LOW, 10000);
  return (freq == 0) ? 9999 : freq;
}

// Simple color detection logic (calibrate as needed)
String detectColor(int red, int green, int blue) {
  if (red < blue && red < green && red < 40) return "Red";
  if (blue < red && blue < green && blue < 95) return "Blue";
  if (green < red && green < blue && green > 105) return "Green";
  return "Unknown";
}
