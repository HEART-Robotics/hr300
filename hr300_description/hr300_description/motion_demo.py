import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller_demo')
        # Parameters can be lists (passed from launch file)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('positions', [0.5, 0.0, -0.5, 0.0])

        joint_names = self.get_parameter('joint_names').value
        positions = self.get_parameter('positions').value

        if len(joint_names) != len(positions):
            raise ValueError("joint_names and positions lists must have the same length")

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.set_joints(joint_names, positions)

    def set_joints(self, joint_names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = positions
        self.publisher.publish(msg)
        self.get_logger().info(f"Set {joint_names} to {positions}")

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
