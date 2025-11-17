import rclpy
from rclpy.node import Node
from math import atan2, sqrt, degrees, radians, sin, cos
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Robot arm lengths (in mm)
        self.l1 = 96.326
        self.l2 = sqrt(24**2 + 128**2)
        self.l3 = 124
        self.l4 = 133.4
        
        # Joint and motor alignment angles
        self.a = degrees(atan2(24, 128))
        self.b = degrees(atan2(128, 24))
        
        # Subscribe to desired pose topic
        self.pose_sub = self.create_subscription(
            Pose,
            'desired_ee_pose',
            self.ik_callback,
            10
        )
        
        # Publish joint solution
        self.joint_pub = self.create_publisher(JointState, 'ik_joint_solution', 10)
        
        self.get_logger().info('Inverse Kinematics Node Ready')
    
    def ik_callback(self, msg):
    
        # Extract position
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        # Set alpha to 0 (horizontal end effector)
        alpha = 0
        
        try:
            # Compute inverse kinematics
            theta1, theta2, theta3, theta4 = self.compute_ik(x, y, z, alpha)
            
            # Create and publish joint state message
            joint_msg = JointState()
            joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
            joint_msg.position = [
                radians(theta1),
                radians(theta2),
                radians(theta3),
                radians(theta4)
            ]
            self.joint_pub.publish(joint_msg)
            
            # Log the solution
            self.get_logger().info('\n' + '=' * 60)
            self.get_logger().info(f'IK Solution for position ({x:.1f}, {y:.1f}, {z:.1f}) mm:\n')
            self.get_logger().info(f'Joint 1: {theta1:7.2f}degrees = {radians(theta1):7.4f} rad\n')
            self.get_logger().info(f'Joint 2: {theta2:7.2f}degrees = {radians(theta2):7.4f} rad\n')
            self.get_logger().info(f'Joint 3: {theta3:7.2f}degrees = {radians(theta3):7.4f} rad\n')
            self.get_logger().info(f'Joint 4: {theta4:7.2f}degrees = {radians(theta4):7.4f} rad\n')
            self.get_logger().info('=' * 60 + '\n')
            
        except Exception as e:
            self.get_logger().error(f'IK computation failed: {str(e)}')
    
    def compute_ik(self, x, y, z, alpha):
        # THETA 1 - Base rotation
        theta1 = degrees(atan2(y, x))
        
        # De-coupling - remove end effector offset
        y0 = y - self.l4 * sin(radians(theta1)) * cos(radians(alpha))
        x0 = x - self.l4 * cos(radians(theta1)) * cos(radians(alpha))
        z0 = z + self.l4 * sin(radians(alpha))
        
        # Variable lengths
        r = sqrt(x0**2 + y0**2)
        d = z0 - self.l1
        p = sqrt(r**2 + d**2)
        
        # THETA 2 - Shoulder joint
        epsilon = degrees(atan2(d, r))
        c2 = (p**2 + r**2 - d**2) / (2 * p * r)
        c2 = max(-1.0, min(1.0, c2))
        phi = degrees(atan2(sqrt(1 - c2**2), c2))
        theta2 = 90 - self.a - (epsilon + phi)
        
        # THETA 3 - Elbow joint
        c3 = (self.l2**2 + self.l3**2 - p**2) / (2 * self.l2 * self.l3)
        c3 = max(-1.0, min(1.0, c3))
        t3 = degrees(atan2(sqrt(1 - c3**2), c3))
        theta3 = 180 - t3 - self.b
        
        # THETA 4 - Wrist joint
        theta4 = alpha - theta2 - theta3
        
        return theta1, theta2, theta3, theta4


def main(args=None):
    rclpy.init(args=args)
    ik_node = InverseKinematicsNode()
    rclpy.spin(ik_node)
    ik_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()