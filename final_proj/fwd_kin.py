import rclpy
from rclpy.node import Node
from math import cos, sin, pi, atan, sqrt
import numpy as np

from final_proj.homogeneous_xforms import H, A, T, d

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String


joint_pos_topic = "/joint_states"


'''
/joint_states
/kinematics_pose
/option
/parameter_events
/rosout
/states
'''

class FwdKinematicsNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.fwd_kin_subscription = self.create_subscription(
            JointState,
            joint_pos_topic,
            self.fwd_kin_callback,
            10)
        self.fwd_kin_subscription  # prevent unused variable warning

        self.fwd_kin_publisher = self.create_publisher(Pose, 'ee_pose', 10)
        self.fwd_kin_publisher

        # Establish variables for the joint values
        self.q = [0,0,0,0,0]

    def fwd_kin_callback(self, msg):
        # print(msg)
        data = msg.position
        # print(data)
        if len(data) != 5:
            raise Exception(f"Expected a list of length 5, got a list of length {len(data)}")
        
        for i in range(len(data)):
            self.q[i] = data[i]
        print(self.q)

        xform = T(4,self.q[0:4])
        
        msg = Pose()
        msg.position = Point()
        
        pos = d(xform)
        
        msg.position.x = pos[0]
        msg.position.y = pos[1]
        msg.position.z = pos[2]
        self.fwd_kin_publisher.publish(msg)

def main(args=None):
    # Set up rcl
    rclpy.init(args=args)

    # Initialize the node
    kinematics_node = FwdKinematicsNode()

    # Start the node
    rclpy.spin(kinematics_node)

    # Clean up node after it's finished
    kinematics_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()