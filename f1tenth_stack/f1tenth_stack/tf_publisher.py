from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class FramePublisher(Node):

    def __init__(self):
        super().__init__('f1tenth_tf_publisher')

        self.br = TransformBroadcaster(self)

        # odom -> base_link 변환
        #t2 = TransformStamped()
        #t2.header.stamp = self.get_clock().now().to_msg()
        #t2.header.frame_id = 'odom'
        #t2.child_frame_id = 'base_link'
        #t2.transform.translation.x = 0.0
        #t2.transform.translation.y = 0.0
        #t2.transform.translation.z = 0.0
        #t2.transform.rotation.x = 0.0
        #t2.transform.rotation.y = 0.0
        #t2.transform.rotation.z = 0.0
        #t2.transform.rotation.w = 1.0
        #self.br.sendTransform(t2)
     
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
        t_laser.transform.translation.z = 0.16
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
        t_imu.transform.translation.x = 0.27   # IMU의 X 위치 (예시값) 원래 0.2 였는데 laser랑 맞아야한다고 생각해서 0.27로 고정했음.
        t_imu.transform.translation.y = 0.0   # IMU의 Y 위치 (예시값)
        t_imu.transform.translation.z = 0.06   # IMU의 Z 위치 (예시값) 원래 0.1 이었는데 imu 6cm 떨어져있어서 0.06으로 함
        t_imu.transform.rotation.x = 0.0      # IMU의 회전 (필요 시 수정)
        t_imu.transform.rotation.y = 0.0
        t_imu.transform.rotation.z = 0.0
        t_imu.transform.rotation.w = 1.0
        self.br.sendTransform(t_imu)
        
        # base_link -> camera 변환 추가
        # d455의 경우 origin point가 왼쪽 IR 위치로 설정되어있음!(depth, leftIR(infra1)등) : camera_link frame이고,
        # rgb 카메라의 경우 camera_color_frame이라고 해서 y축이으로 더 이동한 frame이 따로 있고 나는 그걸 사용함.
        # depth를 사용할 때는 y축 이동을 보거나 camera_link 프레임을 또 쓰거나 해야할듯하다.
        
        #t_rgb_camera = TransformStamped()
        #t_rgb_camera.header.stamp = current_time
        #t_rgb_camera.header.frame_id = 'base_link'
        #t_rgb_camera.child_frame_id = 'camera_color_frame'
        #t_rgb_camera.transform.translation.x = 0.335
        #t_rgb_camera.transform.translation.y = 0.0
        #t_rgb_camera.transform.translation.z = 0.12
        #t_rgb_camera.transform.rotation.x = 0.0
        #t_rgb_camera.transform.rotation.y = 0.0
        #t_rgb_camera.transform.rotation.z = 0.0
        #t_rgb_camera.transform.rotation.w = 1.0
        #self.br.sendTransform(t_rgb_camera)
        
        #t_depth_camera = TransformStamped()
        #t_depth_camera.header.stamp = current_time
        #t_depth_camera.header.frame_id = 'base_link'
        #t_depth_camera.child_frame_id = 'camera_depth_frame'
        #t_depth_camera.transform.translation.x = 0.335
        #t_depth_camera.transform.translation.y = 0.059
        #t_depth_camera.transform.translation.z = 0.12
        #t_depth_camera.transform.rotation.x = 0.0
        #t_depth_camera.transform.rotation.y = 0.0
        #t_depth_camera.transform.rotation.z = 0.0
        #t_depth_camera.transform.rotation.w = 1.0
        #self.br.sendTransform(t_depth_camera)

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

