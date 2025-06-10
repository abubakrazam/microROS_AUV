#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from custom_interfaces.srv import VariableActivate

class MainThrusterControllerNode(Node):
    def __init__(self):
        super().__init__("main_thruster_controller")

        self.thruster1 = 1
        self.thruster2 = 1
        self.thruster3 = 1
        self.thruster4 = 1
        self.thruster5 = 1
        self.thruster6 = 1
        self.thruster7 = 1
        self.thruster8 = 1

        self.joy_thruster1 = 1
        self.joy_thruster2 = 1
        self.joy_thruster3 = 1
        self.joy_thruster4 = 1
        self.joy_thruster5 = 1
        self.joy_thruster6 = 1
        self.joy_thruster7 = 1
        self.joy_thruster8 = 1

        self.stable_thruster1 = 1
        self.stable_thruster2 = 1
        self.stable_thruster3 = 1
        self.stable_thruster4 = 1

        self.depth_thruster1 = 1
        self.depth_thruster2 = 1
        self.depth_thruster3 = 1
        self.depth_thruster4 = 1

        self.heading_thruster5 = 1
        self.heading_thruster6 = 1
        self.heading_thruster7 = 1
        self.heading_thruster8 = 1

        self.position_thruster5 = 1
        self.position_thruster6 = 1
        self.position_thruster7 = 1
        self.position_thruster8 = 1

        self.position_hold_mode_active = False
        self.depth_hold_mode_active = False
        self.heading_stabilization_active = False
        
        self.state_publisher_ = self.create_publisher(Int32, "thrusters_states", 10)
        self.planar_stabilization_states_subscriber_ = self.create_subscription(Int32, "planar_stabilization_states", self.callback_planar_stabilization_states, 10)
        self.heading_stabilization_states_subscriber_ = self.create_subscription(Int32, "heading_stabilization_states", self.callback_heading_stabilization_states, 10)
        self.depth_holding_states_subscriber_ = self.create_subscription(Int32, "depth_holding_states", self.callback_depth_holding_states, 10)
        self.position_holding_states_subscriber_ = self.create_subscription(Int32, "position_holding_states", self.callback_position_holding_states, 10)
        self.joy_states_subscriber_ = self.create_subscription(Int32, "joy_states", self.callback_joy_states, 10)
        self.position_hold_mode_service_ = self.create_service(VariableActivate, "position_hold_mode", self.callback_position_hold_mode)
        self.depth_hold_mode_service_ = self.create_service(VariableActivate, "depth_hold_mode", self.callback_depth_hold_mode)
        self.heading_stabilization_mode_service_ = self.create_service(VariableActivate, "heading_stabilization_mode", self.heading_stabilization_mode)
        self.publishing_frequency_ = 50
        self.state_publishing_timer_ = self.create_timer(1.0/self.publishing_frequency_, self.state_calculator)
        self.get_logger().info("Main thruster controller has been started.")
    
    def callback_planar_stabilization_states(self,msg):
        planar_stabilization_states = str(msg.data)
        self.stable_thruster1 = int(planar_stabilization_states[0])
        self.stable_thruster2 = int(planar_stabilization_states[1])
        self.stable_thruster3 = int(planar_stabilization_states[2])
        self.stable_thruster4 = int(planar_stabilization_states[3])
    
    def callback_heading_stabilization_states(self,msg):
        heading_stabilization_states = str(msg.data)
        self.heading_thruster5 = int(heading_stabilization_states[0])
        self.heading_thruster6 = int(heading_stabilization_states[1])
        self.heading_thruster7 = int(heading_stabilization_states[2])
        self.heading_thruster8 = int(heading_stabilization_states[3])
    
    def callback_depth_holding_states(self,msg):
        depth_holding_states = str(msg.data)
        self.depth_thruster1 = int(depth_holding_states[0])
        self.depth_thruster2 = int(depth_holding_states[1])
        self.depth_thruster3 = int(depth_holding_states[2])
        self.depth_thruster4 = int(depth_holding_states[3])

    def callback_position_holding_states(self,msg):
        position_holding_states = str(msg.data)
        self.position_thruster5 = int(position_holding_states[0])
        self.position_thruster6 = int(position_holding_states[1])
        self.position_thruster7 = int(position_holding_states[2])
        self.position_thruster8 = int(position_holding_states[3])
    
    def callback_joy_states(self,msg):
        joy_states = str(msg.data)
        self.joy_thruster1 = int(joy_states[0])
        self.joy_thruster2 = int(joy_states[1])
        self.joy_thruster3 = int(joy_states[2])
        self.joy_thruster4 = int(joy_states[3])
        self.joy_thruster5 = int(joy_states[4])
        self.joy_thruster6 = int(joy_states[5])
        self.joy_thruster7 = int(joy_states[6])
        self.joy_thruster8 = int(joy_states[7])

    def callback_position_hold_mode(self,request, response):
        self.position_hold_mode_active = request.activate
        self.get_logger().info("Position Hold mode: " + str(request.activate))

        response.success = True 
        return response 
        
    def callback_depth_hold_mode(self,request, response):
        self.depth_hold_mode_active = request.activate
        self.get_logger().info("Depth Hold mode: " + str(request.activate))

        response.success = True 
        return response 

    def heading_stabilization_mode(self,request, response):
        self.heading_stabilization_active = request.activate
        self.get_logger().info("Heading Stabilization mode: " + str(request.activate))

        response.success = True  
        return response  

    def state_calculator(self):
        state = Int32()

        self.thruster1 = self.joy_thruster1 
        self.thruster2 = self.joy_thruster2 
        self.thruster3 = self.joy_thruster3 
        self.thruster4 = self.joy_thruster4 
        self.thruster5 = self.joy_thruster5 
        self.thruster6 = self.joy_thruster6 
        self.thruster7 = self.joy_thruster7 
        self.thruster8 = self.joy_thruster8 

        if ((self.stable_thruster1 != 1) or (self.stable_thruster2 != 1) or (self.stable_thruster3 != 1) or (self.stable_thruster4 != 1)):
            self.thruster1 = self.stable_thruster1
            self.thruster2 = self.stable_thruster2
            self.thruster3 = self.stable_thruster3
            self.thruster4 = self.stable_thruster4
        
        elif (self.depth_hold_mode_active == True) :
            self.thruster1 = self.depth_thruster1
            self.thruster2 = self.depth_thruster2
            self.thruster3 = self.depth_thruster3
            self.thruster4 = self.depth_thruster4

        if (self.heading_stabilization_active == True):
            if ((self.heading_thruster5 != 1) or (self.heading_thruster6 != 1) or (self.heading_thruster7 != 1) or (self.heading_thruster8 != 1)):
                self.thruster5 = self.heading_thruster5
                self.thruster6 = self.heading_thruster6
                self.thruster7 = self.heading_thruster7
                self.thruster8 = self.heading_thruster8

        elif (self.position_hold_mode_active == True):
            self.thruster5 = self.position_thruster5
            self.thruster6 = self.position_thruster6
            self.thruster7 = self.position_thruster7
            self.thruster8 = self.position_thruster8
            
        state.data = (self.thruster1 * 10000000) + (self.thruster2 * 1000000) + (self.thruster3 * 100000) + (self.thruster4 * 10000) + (self.thruster5 * 1000) + (self.thruster6 * 100) +  (self.thruster7 * 10) + (self.thruster8)
        self.state_publisher_.publish(state)
 
def main(args=None):
    rclpy.init(args=args)
    node = MainThrusterControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()