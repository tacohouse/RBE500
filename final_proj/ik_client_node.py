import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetKinematicsPose
import sys, math



class IKClientNode(Node):
    def __init__(self):
        super().__init__('ik_client_node')
        self.client = self.create_client(SetKinematicsPose, 'compute_ik')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        
        self.get_logger().info('IK Client ready')
    
    def send_ik_request(self, x, y, z):
        request = SetKinematicsPose.Request()
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z
        
        self.get_logger().info(f'Requesting IK for position: x={x}, y={y}, z={z}')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.is_planned:
                self.get_logger().info('IK Solution found:')
                for i, (name, angle) in enumerate(zip(response.joint_position.joint_name, 
                                                       response.joint_position.position)):
                    self.get_logger().info(f'  {name}: {angle:.4f} rad')
                return response.joint_position.position
            else:
                self.get_logger().error('IK failed to find solution')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    client_node = IKClientNode()
    
    # Testing 3 different positions (in millimeters)
    test_positions = [
        (283, 0.0, 217),      # Position 1 (home)
        (143, 244, 215),      # Position 2 (-210 0 0)
        (150, -50, 180)       # Position 3
    ]
    
    if len(sys.argv) == 4:
        # If command line args provided, use those
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        client_node.send_ik_request(x, y, z)
    else:
        # Otherwise test all 3 positions
        for i, (x, y, z) in enumerate(test_positions, 1):
            print(f"\n{'='*50}")
            print(f"TEST POSITION {i}")
            print(f"{'='*50}")
            client_node.send_ik_request(x, y, z)
    
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()