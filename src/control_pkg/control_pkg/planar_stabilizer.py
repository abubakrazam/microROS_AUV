#!/usr/bin/env python3

import numpy as np
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
import tf_transformations as tf2
 
class PlanarStabilizerNode(Node):
    def __init__(self):
        super().__init__("planar_stabilizer")

        self.thruster1 = 1
        self.thruster2 = 1
        self.thruster3 = 1
        self.thruster4 = 1

        self.imu_data_subscriber_ = self.create_subscription(PoseStamped, "pose_data", self.callback_IMU_data, 10)
        self.planar_stabilization_states_publisher_ = self.create_publisher(Int32, "planar_stabilization_states", 10)
        self.get_logger().info("Planar stabilization has been started.")
    
    def callback_IMU_data(self, msg):
        q_w, q_x, q_y, q_z = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        R = self.quaternion_to_rotation_matrix(q_w, q_x, q_y, q_z)

        # Extract the axis of the IMU attached frame which faces upwards
        y_axis = R[:, 1] 
        # Test to figure out how the x, y and z corrdinates of the desired frame match with the one above
        x_component, z_component, y_component = y_axis

        self.get_logger().info(f"x: {x_component}, y: {y_component}, z: {z_component}")
        
        tolerance_angle = 90 - 10
        tolerance_distance = math.cos(math.radians(tolerance_angle))

        if (x_component > tolerance_distance) and (y_component > tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 2, 1, 2, 1
            self.get_logger().info("1")
        elif (x_component > tolerance_distance) and (y_component < -tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 1, 3, 1, 3
            self.get_logger().info("2")
        elif (x_component < -tolerance_distance) and (y_component < -tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 3, 1, 3, 1
            self.get_logger().info("3")
        elif (x_component < -tolerance_distance) and (y_component > tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 1, 2, 1, 2
            self.get_logger().info("4")
        elif (x_component > tolerance_distance) and (-tolerance_distance < y_component < tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 2, 3, 2, 3
            self.get_logger().info("5")
        elif (-tolerance_distance < x_component < tolerance_distance) and (y_component < -tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 3, 3, 3, 3
            self.get_logger().info("6")
        elif (x_component < -tolerance_distance) and (-tolerance_distance < y_component < tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 3, 2, 3, 2
            self.get_logger().info("7")
        elif (-tolerance_distance < x_component < tolerance_distance) and (y_component > tolerance_distance):
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 2, 2, 2, 2
            self.get_logger().info("8")
        else:
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 1, 1, 1, 1
            self.get_logger().info("9")
         
        new_msg = Int32()
        new_msg.data = (self.thruster1 * 1000) +  (self.thruster2 * 100) +  (self.thruster3 * 10) +  (self.thruster4)
        self.planar_stabilization_states_publisher_.publish(new_msg)


    def quaternion_to_rotation_matrix(self, w, x, y, z):
        # R = np.array([
        #     [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w),     2 * (x*z + y*w)],
        #     [2 * (x*y + z*w),       1 - 2 * (x**2 + z**2), 2 * (y*z - x*w)],
        #     [2 * (x*z - y*w),       2 * (y*z + x*w),     1 - 2 * (x**2 + y**2)]
        # ])
        
        quaternion = [x, y, z, w]  # tf2 expects (x, y, z, w) order
        R= tf2.quaternion_matrix(quaternion)[:3, :3]  # Extract 3x3 rotation matri
        return np.array(R)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = PlanarStabilizerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()