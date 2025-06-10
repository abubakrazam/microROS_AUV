#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from custom_interfaces.srv import VariableActivate
from sensor_msgs.msg import Joy
 
 
class HeadingStabilizerNode(Node):
    def __init__(self):
        super().__init__("heading_stabilizer")

        self.thruster1 = 1
        self.thruster2 = 1
        self.thruster3 = 1
        self.thruster4 = 1

        self.reference_heading = None
        self.current_heading = 0.0
        self.heading_stabilization_active = False
        self.tolerance = 5.0  # Allowed yaw deviation in degrees
        
        self.imu_data_subscriber_ = self.create_subscription(PoseStamped, "pose_data", self.callback_IMU_data, 10)
        self.joy_subscriber_ = self.create_subscription(Joy, "joy", self.callback_joy, 10)
        self.heading_stabilization_states_publisher_ = self.create_publisher(Int32, "heading_stabilization_states", 10)
        self.heading_stabilization_mode_client_ = self.create_client(VariableActivate, "heading_stabilization_mode")
        # self.heading_stabilization_mode_service_ = self.create_service(VariableActivate, "heading_stabilization_mode_during_position_stabilization", self.heading_stabilization_mode)
        self.publishing_frequency_ = 50
        self.state_publishing_timer_ = self.create_timer(1.0/self.publishing_frequency_, self.control_loop)
        self.get_logger().info("Heading stabilizer has been started.")
    
    # def heading_stabilization_mode(self,request):
    #     self.heading_stabilization_active = request.activate
    #     if (self.heading_stabilization_active == True):
    #         self.reference_heading = self.current_heading  # Set initial heading 

    #         while not self.heading_stabilization_mode_client_.wait_for_service(1.0):
    #             self.get_logger().warn("Waiting for heading stabilization mode service...")
    #         request = VariableActivate.Request()
    #         request.activate = True
    #         self.heading_stabilization_mode_client_.call_async(request)
    #     else:
    #         self.reference_heading = None

    #         while not self.heading_stabilization_mode_client_.wait_for_service(1.0):
    #             self.get_logger().warn("Waiting for heading stabilization mode service...")
    #         request = VariableActivate.Request()
    #         request.activate = False
    #         self.heading_stabilization_mode_client_.call_async(request)
    
    def callback_IMU_data(self, msg):
        # Extract yaw angle from IMU quaternion
        q_w, q_x, q_y, q_z = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 
                         1.0 - 2.0 * (q_y * q_y + q_z * q_z))
        self.current_heading = np.degrees(yaw)  # Convert to degrees
    
    def callback_joy(self, msg):
        if (msg.axes[7] == 1.0 or msg.axes[7] == -1.0 or msg.axes[6] == 1.0 or msg.axes[6] == -1.0):  # Forward, Backward, Left or Right
            if not self.heading_stabilization_active:
                self.heading_stabilization_active = True
                self.reference_heading = self.current_heading  # Set initial heading 

                while not self.heading_stabilization_mode_client_.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for heading stabilization mode service...")
                request = VariableActivate.Request()
                request.activate = True
                self.heading_stabilization_mode_client_.call_async(request)
        elif self.heading_stabilization_active:
            self.reference_heading = None
            self.heading_stabilization_active = False

            while not self.heading_stabilization_mode_client_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for heading stabilization mode service...")
            request = VariableActivate.Request()
            request.activate = False
            self.heading_stabilization_mode_client_.call_async(request)

    def control_loop(self):
        #if self.heading_stabilization_active and self.reference_heading is not None:
        if self.reference_heading is None:
            self.reference_heading = self.current_heading
        else:
            heading_error = self.reference_heading - self.current_heading
            #print("here")
            if abs(heading_error) >= self.tolerance:
                self.get_logger().info("Robot is swaying from direction! Engaging heading stabilization.")
                if heading_error > 0:  # Turning right so need to turn left
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 3, 2, 2, 3
                else:  # Turning left so need to turn right
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 2, 3, 3, 2   
            else:
                self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 1, 1, 1, 1
        
        msg = Int32()
        msg.data = (self.thruster1 * 1000) +  (self.thruster2 * 100) +  (self.thruster3 * 10) +  (self.thruster4)
        self.heading_stabilization_states_publisher_.publish(msg)

 
def main(args=None):
    rclpy.init(args=args)
    node = HeadingStabilizerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()