#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import atan2, sqrt, degrees, radians, sin, cos
from std_srvs.srv import Trigger
import sys


class IKService(Node):
    def __init__(self):
        super().__init__('ik_service')
        
        # Robot arm lengths (in mm)
        self.l1 = 96.326
        self.l2 = sqrt(24**2 + 128**2)
        self.l3 = 124
        self.l4 = 133.4
        
        # Joint and motor alignment angles
        self.a = degrees(atan2(24, 128))
        self.b = degrees(atan2(128, 24))
        
        # Declare parameters for desired position
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        
        # Create service
        self.srv = self.create_service(Trigger, 'compute_ik', self.compute_ik_service)
        
        self.get_logger().info('IK Service ready. Use parameters x, y, z to set desired position.')
    
    def compute_ik_service(self, request, response):
        # Get parameters
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        
        alpha = 0.0
        
        try:
            theta1, theta2, theta3, theta4 = self.compute_ik(x, y, z, alpha)
            
            # Format response
            response.success = True
            response.message = (
                f"IK Solution for position ({x:.1f}, {y:.1f}, {z:.1f}) mm:\n"
                f"  Joint 1: {theta1:7.2f}° = {radians(theta1):7.4f} rad\n"
                f"  Joint 2: {theta2:7.2f}° = {radians(theta2):7.4f} rad\n"
                f"  Joint 3: {theta3:7.2f}° = {radians(theta3):7.4f} rad\n"
                f"  Joint 4: {theta4:7.2f}° = {radians(theta4):7.4f} rad"
            )
            
            self.get_logger().info(f"\n{response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"IK computation failed: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def compute_ik(self, x, y, z, alpha):
        # THETA 1
        theta1 = degrees(atan2(y, x))
        
        # De-coupling
        y0 = y - self.l4 * sin(radians(theta1)) * cos(radians(alpha))
        x0 = x - self.l4 * cos(radians(theta1)) * cos(radians(alpha))
        z0 = z + self.l4 * sin(radians(alpha))
        
        # Variable lengths
        r = sqrt(x0**2 + y0**2)
        d = z0 - self.l1
        p = sqrt(r**2 + d**2)
        
        # THETA 2
        epsilon = degrees(atan2(d, r))
        c2 = (p**2 + r**2 - d**2) / (2 * p * r)
        c2 = max(-1.0, min(1.0, c2))
        phi = degrees(atan2(sqrt(1 - c2**2), c2))
        theta2 = 90 - self.a - (epsilon + phi)
        
        # THETA 3
        c3 = (self.l2**2 + self.l3**2 - p**2) / (2 * self.l2 * self.l3)
        c3 = max(-1.0, min(1.0, c3))
        t3 = degrees(atan2(sqrt(1 - c3**2), c3))
        theta3 = 180 - t3 - self.b
        
        # THETA 4
        theta4 = alpha - theta2 - theta3
        
        return theta1, theta2, theta3, theta4


def main(args=None):
    rclpy.init(args=args)
    ik_service = IKService()
    rclpy.spin(ik_service)
    ik_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()