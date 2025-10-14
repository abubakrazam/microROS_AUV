#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <ESP32Servo.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

byte thrusterPin1 = 23;
byte thrusterPin2 = 22;
byte thrusterPin3 = 21;
byte thrusterPin4 = 19;
byte thrusterPin5 = 18;
byte thrusterPin6 = 5;
byte thrusterPin7 = 4;
byte thrusterPin8 = 2;
Servo thruster1;
Servo thruster2;
Servo thruster3;
Servo thruster4;
Servo thruster5;
Servo thruster6;
Servo thruster7;
Servo thruster8;

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  // Get the received 8-digit command
  int command = msg->data;  
  // eg: command = 10102020
  
  // Convert to 8-character string
  char commandStr[9];
  snprintf(commandStr, sizeof(commandStr), "%08d", command);
  // eg: commandStr = "10102020"

  // Define PWM values
  int pwmValues[8];

  // Compute PWM values in one loop
  for (int i = 0; i < 8; i++) {
      if (commandStr[i] == '2') pwmValues[i] = 1600;
      else if (commandStr[i] == '3') pwmValues[i] = 1400;
      else pwmValues[i] = 1500;
  }

  // Assign PWM values in another loop
  thruster1.writeMicroseconds(pwmValues[0]);
  thruster2.writeMicroseconds(pwmValues[1]);
  thruster3.writeMicroseconds(pwmValues[2]);
  thruster4.writeMicroseconds(pwmValues[3]);
  thruster5.writeMicroseconds(pwmValues[4]);
  thruster6.writeMicroseconds(pwmValues[5]);
  thruster7.writeMicroseconds(pwmValues[6]);
  thruster8.writeMicroseconds(pwmValues[7]);
}

void setup() {
  set_microros_transports();
  thruster1.attach(thrusterPin1);
  thruster2.attach(thrusterPin2);
  thruster3.attach(thrusterPin3);
  thruster4.attach(thrusterPin4);
  thruster5.attach(thrusterPin5);
  thruster6.attach(thrusterPin6);
  thruster7.attach(thrusterPin7);
  thruster8.attach(thrusterPin8);
  thruster1.writeMicroseconds(1500); // send "stop" signal to ESC.
  thruster2.writeMicroseconds(1500);
  thruster3.writeMicroseconds(1500);
  thruster4.writeMicroseconds(1500);
  thruster5.writeMicroseconds(1500);
  thruster6.writeMicroseconds(1500);
  thruster7.writeMicroseconds(1500);
  thruster8.writeMicroseconds(1500);
  delay(7000); // delay to allow the ESC to recognize the stopped signal

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "micro_ros_esp32_thruster_controller", "", &support);
  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "thrusters_states");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin(&executor);
}
