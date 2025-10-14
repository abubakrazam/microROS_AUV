// ================================================================
// === micro-ROS Servo Motor Example (ESP32) ======================
// ================================================================
// This example demonstrates how to:
// 1. Connect an ESP32 to a ROS 2 system using micro-ROS.
// 2. Use a timer callback to toggle a servo motor between 0° and 180°.
// 3. Show LED blinking on error (debug feedback).
//
// Author: Abu Bakr Azam
// Target Board: ESP32 (any micro-ROS compatible variant)
// ================================================================


// ----------------------------
// Include Required Libraries
// ----------------------------

// micro-ROS core library (connects ESP32 to ROS 2)
#include <micro_ros_arduino.h>

// ROS 2 core client libraries
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ESP32 servo library (controls standard RC servo motors)
#include <ESP32Servo.h>

// ROS 2 standard message type (not used to publish yet, but included for extension)
#include <std_msgs/msg/float32.h>


// ----------------------------
// Declare Global ROS 2 Variables
// ----------------------------

// ROS 2 data structures
std_msgs__msg__Float32 msg;       // Example message type
rclc_executor_t executor;         // Executor: runs callbacks
rclc_support_t support;           // ROS 2 support context
rcl_allocator_t allocator;        // Memory manager
rcl_node_t node;                  // ROS 2 node handle
rcl_timer_t timer;                // Timer that triggers servo motion


// ----------------------------
// Hardware Configuration
// ----------------------------

#define SERVO_PIN 27              // Servo control pin (PWM output)
#define LED_PIN 13                // On-board LED (for debugging/error state)


// ----------------------------
// Variables for Servo Control
// ----------------------------

static unsigned long lastServoFlip = 0;  // Used for time tracking (optional)
int posDegrees = 0;                      // Current servo position (in degrees)
Servo servo;                             // Servo object


// ----------------------------
// Error Handling Macros
// ----------------------------
// These simplify error checking for ROS 2 calls.

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// ----------------------------
// Error Handling Function
// ----------------------------
// If a ROS 2 initialization error occurs, blink LED rapidly forever.

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // toggle LED
    delay(100); // blink interval
  }
}


// ----------------------------
// Timer Callback Function
// ----------------------------
// This function runs periodically, based on the timer setting in setup().
// Here, it flips the servo between 0° and 180° every second.

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time); // Avoid compiler warning for unused variable

  if (timer != NULL) {

    // Toggle servo position: if 0°, go to 180°; if 180°, go back to 0°
    if (posDegrees == 0) {
      posDegrees = 180;
    } else {
      posDegrees = 0;
    }

    // Move the servo to the new position
    servo.write(posDegrees);

    // Optional: short delay for servo movement stability
    delay(50);

    // (You can add Serial.println(posDegrees); for debugging if needed)
  }
}


// ================================================================
// === SETUP FUNCTION ==============================================
// ================================================================
// Runs once at startup. Initializes micro-ROS, timer, executor, and servo.

void setup() 
{
  // Initialize communication between ESP32 and micro-ROS Agent
  set_microros_transports();

  // Attach servo signal to defined pin
  servo.attach(SERVO_PIN);

  // Optional: set LED pin mode for visual feedback
  pinMode(LED_PIN, OUTPUT);

  // Use the default allocator for ROS 2 memory management
  allocator = rcl_get_default_allocator();

  // ---------------------------------------------------------------
  // Step 1: Initialize ROS 2 support (core setup)
  // ---------------------------------------------------------------
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // ---------------------------------------------------------------
  // Step 2: Create a ROS 2 node
  // ---------------------------------------------------------------
  // The node represents your ESP32 in the ROS 2 network.
  RCCHECK(rclc_node_init_default(
    &node,
    "micro_ros_arduino_node",   // Node name
    "",                         // Namespace (optional)
    &support));

  // ---------------------------------------------------------------
  // Step 3: Create a Timer
  // ---------------------------------------------------------------
  // This timer triggers the servo flip every 1000 ms (1 second)
  const unsigned int timer_timeout = 1000; // milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout), // Convert ms → nanoseconds
    timer_callback));            // Function to call every time

  // ---------------------------------------------------------------
  // Step 4: Initialize Executor
  // ---------------------------------------------------------------
  // The executor runs callbacks (e.g., timers, subscriptions)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Optional: Initialize message data
  msg.data = 0;
}


// ================================================================
// === LOOP FUNCTION ===============================================
// ================================================================
// Continuously runs after setup().
// The executor checks if it's time to call any callbacks (like our timer).

void loop() {
  delay(100); // Light delay to prevent CPU overload

  // Run any pending callbacks (timer, subscription, etc.)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
