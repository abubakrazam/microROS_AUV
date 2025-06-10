#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32


class JoyControllerNode(Node): 
    def __init__(self):
        super().__init__("joy_controller") 
        self.thrusters_states_ = 11111111
        self.joy_subscriber_ = self.create_subscription(Joy, "joy", self.callback_joy, 10)
        self.joy_states_publisher_ = self.create_publisher(Int32, "joy_states", 10)
        self.get_logger().info("Joy controller has been started.")

    def callback_joy(self, msg):
        if (msg.buttons[3] == 1):       # Up
            self.thrusters_states_ = 23321111
        elif (msg.buttons[0] == 1):     # Down
            self.thrusters_states_ = 32231111
        elif (msg.axes[7] == 1.0):     # Forward 
            self.thrusters_states_ = 11113232
        elif (msg.axes[7] == -1.0):     # Backward 
            self.thrusters_states_ = 11112323
        elif (msg.axes[6] == 1.0):      # Left
            self.thrusters_states_ = 11113333
        elif (msg.axes[6] == -1.0):     # Right
            self.thrusters_states_ = 11112222
        elif (msg.buttons[1] == 1):      # Rotate right 
            self.thrusters_states_ = 11112332
        elif (msg.buttons[2] == 1):     # Rotate left
            self.thrusters_states_ = 11113223
        else:
            self.thrusters_states_ = 11111111
        
        new_msg = Int32()
        new_msg.data = self.thrusters_states_
        self.joy_states_publisher_.publish(new_msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = JoyControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()