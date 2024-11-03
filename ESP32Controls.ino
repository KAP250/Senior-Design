#include <ESP32Servo.h>

#include <micro_ros_arduino.h> 
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h> //Using Twist data structure

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg; //Received Twist
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//Servo setup vars
Servo myservo;
int pos = 0;    // variable to store the servo position
int servoPin = 13;

//ESC setup vars
Servo esc;
int escPin = 12;

//Variables to store previous data
float last_angular_z = 999.0;

unsigned long last_msg_time = 0; // Last time a valid message was processed
const unsigned long debounce_interval = 100; // Minimum time in milliseconds

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() 
{
  while (1) 
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin) 
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Only proceed if debounce interval has passed
  unsigned long current_time = millis();
  if (current_time - last_msg_time < debounce_interval) 
  {
    return;
  }
  last_msg_time = current_time;

  // Check if the received angular.z value is the same as the last one
  if (msg->angular.z == last_angular_z) 
  {
    return;
  }
  
  // Update the last_angular_z with the new value
  last_angular_z = msg->angular.z;

  // Perform actions based on the new angular.z value
  if(msg->angular.z == 0.0) 
  {
    straight();
  } 
  else if(msg->angular.z == 30.0) 
  {
    turnRight();
  } 
  else if(msg->angular.z == -30.0) 
  {
    turnLeft();
  }
}

void turnRight()
{
  myservo.write(180);
  delay(50);
}

void turnLeft()
{
  myservo.write(10);
  delay(50);
}

void straight()
{
  myservo.write(90);
  delay(50);
}

void setup() 
{
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(1000);
  
  Serial.begin(115200);  // Initialize Serial for output

  // Setup for Servo and ESC
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400); //Setup for Servo
  esc.setPeriodHertz(50);
  esc.attach(escPin, 500, 2400); //Setup for ESC

  // Initialize with a neutral signal
  esc.writeMicroseconds(1500);

  //Setup for MicroROS below
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "micro_ros_arduino_node"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); //1 for number of tasks(Only subscribing for data)
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() 
{
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

