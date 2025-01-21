import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class CmdVelToDrive(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_drive')
        
        # Subscription to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for /drive
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        self.get_logger().info('CmdVelToDrive Node started.')

    def cmd_vel_callback(self, msg: Twist):
        # Convert Twist to AckermannDriveStamped
        drive_msg = AckermannDriveStamped()
        
        # Header information
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        
        # Populate AckermannDrive fields
        drive_msg.drive.speed = msg.linear.x
        drive_msg.drive.steering_angle = msg.angular.z
        
        # Publish the drive message
        self.publisher.publish(drive_msg)
        self.get_logger().info(f'Published drive message: speed={msg.linear.x}, steering_angle={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down CmdVelToDrive node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
