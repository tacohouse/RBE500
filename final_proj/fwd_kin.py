import rclpy
from rclpy.node import Node
from math import cos, sin, pi, atan, sqrt
import numpy as np

from final_proj.homogeneous_xforms import T, d, R

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
        q = Quaternion()
        
        pos = d(xform)
        rot = R(xform)
        
        msg.position.x = pos[0]
        msg.position.y = pos[1]
        msg.position.z = pos[2]

        # Now, need to calculate the Quaternion representation of the matrix xform
        # Code found here: https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
        
        t = 0
        if rot[2,2] < 0:
            if rot[0,0] > rot[1,1]:
                t = 1 + rot[0,0] - rot[1,1] - rot[2,2]
                q.x = t
                q.y = rot[1,0] + rot[0,1]
                q.z = rot[2,0] + rot[0,2]
                q.w = rot[1,2] + rot[2,1]
            else:
                t = 1 - rot[0,0] + rot[1,1] - rot[2,2]
                q.x = rot[0,1] + rot[1,0]
                q.y = t
                q.z = rot[1,2] + rot[2,1]
                q.w = rot[2,0] - rot[0,2]
        else:
            if rot[0,0] < -rot[1,1]:
                t = 1 - rot[0,0] - rot[1,1] + rot[2,2]
                q.x = rot[2,0] + rot[0,2]
                q.y = rot[1,2] + rot[2,1]
                q.z = t
                q.w = rot[0,1] - rot[1,0]
            else:
                t = 1 + rot[0,0] + rot[1,1] + rot[2,2]
                q.x = rot[1,2] - rot[2,1]
                q.y = rot[2,0] - rot[0,2]
                q.z = rot[0,1] - rot[1,0]
                q.w = t
        q.x = q.x * 0.5 / sqrt(t)
        q.y = q.y * 0.5 / sqrt(t)
        q.z = q.z * 0.5 / sqrt(t)
        q.w = q.w * 0.5 / sqrt(t)

        msg.orientation = q
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