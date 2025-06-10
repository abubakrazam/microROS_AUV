#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from custom_interfaces.srv import VariableActivate
from std_msgs.msg import Int32
 
class PositionHolderNode(Node):
    def __init__(self):
        super().__init__("position_holder")

        self.thruster1 = 1
        self.thruster2 = 1
        self.thruster3 = 1
        self.thruster4 = 1

        self.position_hold_mode_active = True
        self.xyz = True
        self.current_x = 0.0
        self.current_y = 0.0
        self.reference_x = None
        self.reference_y = None
        self.tolerance = 0.05  # Allowed deviation in Metres

        self.sonar_data_subscriber_ = self.create_subscription(PoseStamped, "pose_data", self.callback_sonar_data, 10)
        self.joy_subscriber_ = self.create_subscription(Joy, "joy", self.callback_joy, 10)
        self.position_holding_states_publisher_ = self.create_publisher(Int32, "position_holding_states", 10)
        self.position_hold_mode_client_ = self.create_client(VariableActivate, "position_hold_mode")
        # self.heading_stabilization_mode_client_ = self.create_client(VariableActivate, "heading_stabilization_mode_during_position_stabilization")
        self.publishing_frequency_ = 50
        self.state_publishing_timer_ = self.create_timer(1.0/self.publishing_frequency_, self.control_loop)
        self.get_logger().info("Position holder has been started.")

    # def callback_joy(self, msg):
    #     if (msg.buttons[5] == 1):
    #         self.get_logger().info("button pressed.")
    #         if not self.position_hold_mode_active:
    #             self.position_hold_mode_active = True
    #             self.reference_x = self.current_x  
    #             self.reference_y = self.current_y  

    #             while not self.position_hold_mode_client_.wait_for_service(1.0):
    #                 self.get_logger().warn("Waiting for position hold mode service...")
    #             request = VariableActivate.Request()
    #             request.activate = True
    #             self.position_hold_mode_client_.call_async(request)
    #         else:
    #             self.reference_x = None
    #             self.reference_y = None
    #             self.position_hold_mode_active = False
                
    #             while not self.position_hold_mode_client_.wait_for_service(1.0):
    #                 self.get_logger().warn("Waiting for position hold mode service...")
    #             request = VariableActivate.Request()
    #             request.activate = False
    #             self.position_hold_mode_client_.call_async(request)

    def callback_sonar_data(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        if self.xyz == True:
            self.reference_x = self.current_x
            self.reference_y = self.current_y
        self.xyz = False

    
    def control_loop(self):
        if self.position_hold_mode_active and self.reference_x is not None:
            x_error = self.reference_x - self.current_x
            y_error = self.reference_y - self.current_y

            if abs(x_error) >= self.tolerance:
                # while not self.heading_stabilization_mode_client_.wait_for_service(1.0):
                #     self.get_logger().warn("Waiting for position hold mode service...")
                # request = VariableActivate.Request()
                # request.activate = True
                # self.heading_stabilization_mode_client_.call_async(request)
                if x_error > 0:  # Robot is right so go left
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 3, 3, 3, 3
                else:  # Robot is left so go right
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 2, 2, 2, 2 

            elif abs(y_error) >= self.tolerance:
                # while not self.heading_stabilization_mode_client_.wait_for_service(1.0):
                #     self.get_logger().warn("Waiting for position hold mode service...")
                # request = VariableActivate.Request()
                # request.activate = True
                # self.heading_stabilization_mode_client_.call_async(request)
                if y_error > 0:  # Robot is front so go backward
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 2, 3, 2, 3
                else:  # Robot is back so go forward
                    self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 3, 2, 3, 2 
            # else:
            #     while not self.heading_stabilization_mode_client_.wait_for_service(1.0):
            #         self.get_logger().warn("Waiting for position hold mode service...")
            #     request = VariableActivate.Request()
            #     request.activate = False
            #     self.heading_stabilization_mode_client_.call_async(request)
        else:
            self.thruster1, self.thruster2, self.thruster3, self.thruster4 = 1, 1, 1, 1
        
        msg = Int32()
        msg.data = (self.thruster1 * 1000) +  (self.thruster2 * 100) +  (self.thruster3 * 10) +  (self.thruster4)
        self.position_holding_states_publisher_.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = PositionHolderNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()