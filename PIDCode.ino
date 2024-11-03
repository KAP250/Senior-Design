#include <ESP32Servo.h>


Servo myservo;
const int targetPosition = 89;  // Your desired target position (in degrees)
const int servoPin = 18;        // Connect the servo signal wire to GPIO 2
const float dt = 0.02;         // Time step (adjust as needed)
const float Kp = 0.5;          // Proportional gain
const float Ki = 0.1;          // Integral gain
const float Kd = 0.01;         // Derivative gain

float integral = 0;
float previousError = 0;

void setup() {
  Serial.begin(115200);
  myservo.attach(servoPin);
  myservo.write(0);  // Initialize the servo to the middle position
  delay(500);
}

void loop() {
  float currentPosition = myservo.read();  // Read the current position of the servo
  float error = targetPosition - currentPosition;
  
  // Calculate the PID components
  float proportional = Kp * error;
  integral += Ki * error * dt;
  float derivative = Kd * (error - previousError) / dt;
  
  // Calculate the control output
  float output = proportional + integral + derivative;

  // Display Results on Serial Monitor
  Serial.print(currentPosition);
  Serial.print(" ");
  Serial.print(error);
  Serial.print("   ");

  Serial.print(proportional);
  Serial.print(" ");
  Serial.print(integral);
  Serial.print(" ");
  Serial.print(derivative);
  Serial.print("   ");
  
  
  Serial.print(output);
  Serial.println(" ");
  
  // Apply the control output to the servo
  myservo.write(currentPosition + output);
  previousError = error;
  
  delay(dt * 1000);  // Convert time step to milliseconds and wait
}