// ================================================================
// === micro-ROS Ultrasonic Sensor Publisher (ESP32) ==============
// ================================================================
// This example demonstrates how to:
// 1. Connect an ESP32 to a ROS 2 system using micro-ROS.
// 2. Read distance data from an ultrasonic sensor (HC-SR04).
// 3. Publish that distance as a ROS 2 topic of type std_msgs/msg/Float32.
//
// Author: Abu Bakr Azam
// Target Board: ESP32 (any variant supported by micro-ROS Arduino)
// ================================================================


// ----------------------------
// Include Required Libraries
// ----------------------------

// micro-ROS Arduino library (handles communication with ROS 2)
#include <micro_ros_arduino.h>

// Standard C and ROS 2 client support libraries
#include <stdio.h>
#include <rcl/rcl.h>              // Core ROS 2 client library
#include <rcl/error_handling.h>   // Error handling for ROS 2 operations
#include <rclc/rclc.h>            // ROS 2 Client Support C API (simplified)
#include <rclc/executor.h>        // Executor to run callbacks

// Standard ROS 2 message type: Float32
#include <std_msgs/msg/float32.h>


// ----------------------------
// Declare Global ROS 2 Variables
// ----------------------------

std_msgs__msg__Float32 msg;       // The ROS message to be published
rcl_publisher_t publisher;        // Publisher handle
rclc_executor_t executor;         // Executor to manage callbacks
rclc_support_t support;           // ROS 2 support context
rcl_allocator_t allocator;        // Memory allocator
rcl_node_t node;                  // ROS 2 node handle
rcl_timer_t timer;                // Timer handle


// ----------------------------
// Define Pin Connections
// ----------------------------

#define LED_PIN 13                // On-board LED for debugging

#define TRIG_PIN 25               // Ultrasonic sensor TRIG pin
#define ECHO_PIN 26               // Ultrasonic sensor ECHO pin
#define SOUND_SPEED 0.034         // Speed of sound in cm/µs (for distance calc)


// ----------------------------
// Define Measurement Variables
// ----------------------------

float latest_distance_cm;         // Most recent distance measurement
long duration;                    // Duration of echo pulse (µs)
static unsigned long lastServoFlip = 0; // (Unused here but kept for reference)


// ----------------------------
// Error Handling Macros
// ----------------------------
// RCCHECK  → for critical errors (halts program)
// RCSOFTCHECK → for non-critical errors (continues)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// ----------------------------
// Error Loop Function
// ----------------------------
// Blinks the LED rapidly if a critical ROS 2 setup error occurs.

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


// ----------------------------
// Timer Callback Function
// ----------------------------
// This function runs every time the ROS 2 timer triggers.
// It measures distance using the ultrasonic sensor and publishes it.

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);  // Avoid unused variable warning

  if (timer != NULL) {

    // 1. Trigger the ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 2. Measure the echo duration
    duration = pulseIn(ECHO_PIN, HIGH);

    // 3. Convert duration to distance (in cm)
    latest_distance_cm = duration * SOUND_SPEED / 2.0;

    // 4. Fill ROS message and publish
    msg.data = latest_distance_cm;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}


// ================================================================
// === SETUP FUNCTION ==============================================
// ================================================================
// Runs once when the ESP32 starts up.
// Initializes the micro-ROS node, publisher, timer, and executor.

void setup() {

  // Initialize micro-ROS transport layer (e.g., serial, WiFi, etc.)
  set_microros_transports();

  // Initialize default values
  latest_distance_cm = 0;
  duration = 0;

  // Configure pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); 
  pinMode(LED_PIN, OUTPUT);

  delay(2000);  // Give some time for ROS agent to start (if needed)

  // 1. Get default allocator (used for memory management)
  allocator = rcl_get_default_allocator();

  // 2. Initialize ROS 2 support context (must be done first)
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // 3. Create a ROS 2 node (acts as the ESP32 identity)
  RCCHECK(rclc_node_init_default(
    &node,
    "micro_ros_arduino_node",  // Node name
    "",                        // Namespace (optional)
    &support));

  // 4. Create a publisher (for topic "micro_ros_arduino_node_publisher")
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_arduino_node_publisher"));

  // 5. Create a timer (runs every 1000 ms = 1 second)
  const unsigned int timer_timeout = 1000; // milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // 6. Create executor (handles callback execution)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize message data
  msg.data = 0;
}


// ================================================================
// === LOOP FUNCTION ===============================================
// ================================================================
// Runs continuously after setup().
// The executor checks if any timers or callbacks are ready to run.

void loop() {

  delay(100); // Short delay to prevent CPU overuse

  // Spin executor to handle any pending callbacks (e.g., timer)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}

