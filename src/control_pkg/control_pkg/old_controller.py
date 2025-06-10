#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
 
 
class Esp32ThrusterControllerNode(Node): 
    def __init__(self):
        super().__init__("esp32_thruster_controller") 
        self.thrusters_states_ = 0
        self.joystick_subscriber_ = self.create_subscription(Joy, "joy", self.callback_joy, 10)
        self.thrusters_states_publisher_ = self.create_publisher(Int32, "thrusters_states", 10)
        self.get_logger().info("ESP32 thruster controller has been started.")

    def callback_joy(self, msg):
        if (msg.buttons[3] == 1):
            self.thrusters_states_ = 1
        elif (msg.buttons[0] == 1):
            self.thrusters_states_ = 2
        elif (msg.buttons[1] == 1):
            self.thrusters_states_ = 3
        elif (msg.buttons[2] == 1):
            self.thrusters_states_ = 4
        elif (msg.axes[7] == 1.0):
            self.thrusters_states_ = 5
        elif (msg.axes[7] == -1.0):
            self.thrusters_states_ = 6
        elif (msg.axes[6] == 1.0):
            self.thrusters_states_ = 7
        elif (msg.axes[6] == -1.0):
            self.thrusters_states_ = 8
        else:
            self.thrusters_states_ = 0
        new_msg = Int32()
        new_msg.data = self.thrusters_states_
        self.thrusters_states_publisher_.publish(new_msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Esp32ThrusterControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()