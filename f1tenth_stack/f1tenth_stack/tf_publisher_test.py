from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class FramePublisher(Node):

    def __init__(self):
        super().__init__('f1tenth_tf_publisher')

        self.br = TransformBroadcaster(self)     
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()

        # base_link -> laser 변환
        t_laser = TransformStamped()
        t_laser.header.stamp = current_time
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser'
        t_laser.transform.translation.x = 0.27
        t_laser.transform.translation.y = 0.0
        t_laser.transform.translation.z = 0.11
        t_laser.transform.rotation.x = 0.0
        t_laser.transform.rotation.y = 0.0
        t_laser.transform.rotation.z = 0.0
        t_laser.transform.rotation.w = 1.0
        self.br.sendTransform(t_laser)

        # base_link -> imu 변환 추가
        t_imu = TransformStamped()
        t_imu.header.stamp = current_time
        t_imu.header.frame_id = 'base_link'
        t_imu.child_frame_id = 'imu'
        t_imu.transform.translation.x = 0.2   # IMU의 X 위치 (예시값)
        t_imu.transform.translation.y = 0.0   # IMU의 Y 위치 (예시값)
        t_imu.transform.translation.z = 0.1   # IMU의 Z 위치 (예시값)
        t_imu.transform.rotation.x = 0.0      # IMU의 회전 (필요 시 수정)
        t_imu.transform.rotation.y = 0.0
        t_imu.transform.rotation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        self.br.sendTransform(t_imu)
        
        # odom -> base_link 변환
        t2 = TransformStamped()
        t2.header.stamp = current_time
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.br.sendTransform(t2)

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

