// ================================================================
// === micro-ROS: Ultrasonic + Servo Control Example (ESP32) =======
// ================================================================
// This example demonstrates how to:
//
// 1. Use an **ultrasonic sensor** (HC-SR04) to measure distance.
// 2. **Publish** that distance to ROS 2 as a Float32 topic.
// 3. **Subscribe** to a Float32 topic (e.g., threshold command).
// 4. **Move a servo motor** when the subscribed threshold condition is met.
// 5. Handle ROS 2 communication through micro-ROS on an ESP32.
//
// Author: Abu Bakr Azam
// Board: ESP32
// ================================================================


// ----------------------------
// Include Required Libraries
// ----------------------------

#include <micro_ros_arduino.h>     // micro-ROS communication layer
#include <stdio.h>                 // Standard C I/O library
#include <rcl/rcl.h>               // Core ROS 2 client support
#include <rcl/error_handling.h>    // ROS 2 error handling
#include <rclc/rclc.h>             // ROS 2 micro client convenience layer
#include <rclc/executor.h>         // ROS 2 executor for callbacks
#include <ESP32Servo.h>            // Servo motor control library for ESP32
#include <std_msgs/msg/float32.h>  // Standard ROS 2 message type (float32)


// ----------------------------
// ROS 2 Global Declarations
// ----------------------------

// ROS 2 entities
rcl_subscription_t subscriber;     // Receives messages from ROS 2
rcl_publisher_t publisher;         // Sends sensor data to ROS 2
rcl_timer_t timer;                 // Periodic timer to trigger publishing
rcl_node_t node;                   // The ROS 2 node (represents this device)
rclc_executor_t executor;          // Handles all callbacks
rclc_support_t support;            // ROS 2 communication support
rcl_allocator_t allocator;         // Memory management

// ROS 2 message variable (for sending & receiving float values)
std_msgs__msg__Float32 msg;


// ----------------------------
// Hardware Configuration
// ----------------------------

#define TRIG_PIN 25        // Ultrasonic trigger pin
#define ECHO_PIN 26        // Ultrasonic echo pin
#define SERVO_PIN 27       // Servo signal pin
#define LED_PIN 13         // LED pin (for debug/error indication)
#define SOUND_SPEED 0.034  // Sound speed in cm/µs (for distance calc)


// ----------------------------
// Variables
// ----------------------------

float latest_distance_cm;  // Holds last measured distance
long duration;             // Time for echo pulse
int posDegrees = 0;        // Servo position (0 or 180 degrees)
Servo servo;               // Servo object instance


// ----------------------------
// Helper Macros for Error Checking
// ----------------------------
// RCCHECK: stops program if a ROS 2 function fails
// RCSOFTCHECK: ignores minor errors (non-fatal)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// ----------------------------
// Error Loop (Debug)
// ----------------------------
// If a setup or runtime error occurs, blink LED rapidly forever.

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


// ----------------------------
// Timer Callback
// ----------------------------
// Runs every 1000 ms (set in setup()).
// Publishes the latest ultrasonic distance to ROS 2 topic.

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  
  if (timer != NULL) {
    // Publish latest distance value
    msg.data = latest_distance_cm;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}


// ----------------------------
// Subscription Callback
// ----------------------------
// Receives messages from a Float32 topic.
// If the received value (e.g., threshold) < 20 cm, toggle servo.

void subscription_callback(const void * msgin) {  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  // Example logic: move servo if received threshold is below 20
  if (msg->data < 20) {
    // Toggle servo position
    if (posDegrees == 0) {
      posDegrees = 180;
    } else {
      posDegrees = 0;
    }
    servo.write(posDegrees);
    delay(5);  // Short pause for smooth movement
  }
}


// ================================================================
// === SETUP FUNCTION ==============================================
// ================================================================

void setup() {
  // Initialize micro-ROS serial transport (USB/Serial)
  set_microros_transports();

  // Initialize sensor and servo pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  servo.attach(SERVO_PIN);

  // Initialize variables
  latest_distance_cm = 0;
  duration = 0;
  delay(2000);  // Small delay for hardware stabilization

  // Get default memory allocator
  allocator = rcl_get_default_allocator();

  // ---------------------------------------------------------------
  // 1. Initialize micro-ROS support
  // ---------------------------------------------------------------
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // ---------------------------------------------------------------
  // 2. Create a ROS 2 Node
  // ---------------------------------------------------------------
  RCCHECK(rclc_node_init_default(
    &node,
    "micro_ros_arduino_node",  // Node name
    "",                        // Namespace
    &support));

  // ---------------------------------------------------------------
  // 3. Create a Publisher (distance readings)
  // ---------------------------------------------------------------
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "ultrasonic_distance"));   // Topic name for publishing

  // ---------------------------------------------------------------
  // 4. Create a Subscription (threshold commands)
  // ---------------------------------------------------------------
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "ultrasonic_distance"));          // Topic name for subscribing

  // ---------------------------------------------------------------
  // 5. Create a Timer (1-second interval)
  // ---------------------------------------------------------------
  const unsigned int timer_timeout = 1000; // in milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout), // convert ms to ns
    timer_callback));

  // ---------------------------------------------------------------
  // 6. Create an Executor
  // ---------------------------------------------------------------
  // The executor handles both timer and subscription callbacks
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // Initialize message
  msg.data = 0;
}


// ================================================================
// === LOOP FUNCTION ===============================================
// ================================================================
// Continuously reads ultrasonic distance and runs ROS callbacks.

void loop() {
  // Trigger ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time (with timeout)
  duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // Calculate distance (cm)
  latest_distance_cm = duration * SOUND_SPEED / 2;

  // Run ROS 2 callbacks (timer + subscription)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
