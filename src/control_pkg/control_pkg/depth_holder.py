#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from custom_interfaces.srv import VariableActivate
from std_msgs.msg import Int32
 
 
class DepthHolderNode(Node):
    def __init__(self):
        super().__init__("depth_holder")

        self.thruster1 = 1
        self.thruster2 = 1
        self.thruster3 = 1
        self.thruster4 = 1

        self.depth_hold_mode_active = False
        self.current_depth = 0.0
        self.reference_depth = None
        self.tolerance = 0.05  # Allowed depth deviation in Metres

        self.sonar_data_subscriber_ = self.create_subscription(PoseStamped, "pose_data", self.callback_sonar_data, 10)
        self.joy_subscriber_ = self.create_subscription(Joy, "joy", self.callback_joy, 10)
        self.depth_holding_states_publisher_ = self.create_publisher(Int32, "depth_holding_states", 10)
        self.depth_hold_mode_client_ = self.create_client(VariableActivate, "depth_hold_mode")
        self.publishing_frequency_ = 50
        self.state_publishing_timer_ = self.create_timer(1.0/self.publishing_frequency_, self.control_loop)
        self.get_logger().info("Depth holder has been started.")

    def callback_joy(self, msg):
        if (msg.buttons[4] == 1):
            if not self.depth_hold_mode_active:
                self.depth_hold_mode_active = True
                self.reference_depth = self.current_depth  # Set initial depth      

                while not self.depth_hold_mode_client_.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for depth hold mode service...")
                request = VariableActivate.Request()
                request.activate = True
                self.depth_hold_mode_client_.call_async(request)
            else:
                self.reference_depth = None
                self.depth_hold_mode_active = False
                
                while not self.depth_hold_mode_client_.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for depth hold mode service...")
                request = VariableActivate.Request()
                request.activate = False
                self.depth_hold_mode_client_.call_async(request)

    def callback_sonar_data(self, msg):
        z = msg.pose.position.z
        self.current_depth = -z

    def control_loop(self):
        if self.depth_hold_mode_active and self.reference_depth is not None:
            depth_error = self.reference_depth - self.current_depth

            if abs(depth_error) >= self.tolerance:
                if depth_error > 0:  # Robot is down so go up
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 2, 3, 3, 2
                else:  # Robot is up so go down
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 3, 2, 2, 3   
        else:
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 1, 1, 1, 1
        
        msg = Int32()
        msg.data = (self.thruster1 * 1000) +  (self.thruster2 * 100) +  (self.thruster3 * 10) +  (self.thruster4)
        self.depth_holding_states_publisher_.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = DepthHolderNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()