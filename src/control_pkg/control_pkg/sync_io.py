import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

import csv
from datetime import datetime

class LatestStateLogger(Node):
    def __init__(self):
        super().__init__('latest_state_logger')

        self.pose_sub = self.create_subscription(PoseStamped, '/pose_data', self.pose_callback, 10)
        self.state_sub = self.create_subscription(Int32, '/heading_stabilization_states', self.state_callback, 10)

        self.latest_state = None

        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f'simple_synced_output_{now_str}.csv'
        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'timestamp', 'pose_x', 'pose_y', 'pose_z',
            'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
            'latest_state'
        ])
        self.get_logger().info(f"Writing to {self.csv_path}")

    def state_callback(self, msg: Int32):
        self.latest_state = msg.data

    def pose_callback(self, pose_msg: PoseStamped):
        if self.latest_state is None:
            return  # Wait until we get a state

        ts = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
        pose = pose_msg.pose

        row = [
            ts,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            self.latest_state
        ]
        self.writer.writerow(row)

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LatestStateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
