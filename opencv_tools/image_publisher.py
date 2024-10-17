import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2
import numpy as np

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)

        # Picamera2 yapılandırması, çözünürlüğü ayarla (640x480) ve FPS'yi 30 yap
        self.picam2 = Picamera2()
        camera_config = self.picam2.create_video_configuration(
            main={"size": (640, 480)}, 
            controls={"FrameDurationLimits": (33333, 33333)}  # 30 FPS ayarı
        )
        self.picam2.configure(camera_config)

        # CvBridge nesnesi
        self.br = CvBridge()

        # Kamerayı başlat
        self.picam2.start()

        # ORB-SLAM3'teki distortion parametreleri
        self.distortion_params = np.array([0.262383, -0.953104, -0.005358, 0.002628, 1.16331])

        # Kamera içsel parametreleri (fx, fy, cx, cy)
        self.camera_matrix = np.array([[517.306, 0, 318.643],
                                       [0, 516.469, 255.314],
                                       [0, 0, 1]])

        # 10 Hz frekansla frame'leri yayınla
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30 FPS -> 1/30 = 0.033 saniye

    def timer_callback(self):
        frame = self.picam2.capture_array()

        # Eğer 4 kanallı (RGBA) bir görüntü ise, bunu BGR formatına çevir
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        # Bozulmayı (distortion) düzelt
        frame = cv2.undistort(frame, self.camera_matrix, self.distortion_params)

        # OpenCV formatındaki frame'i ROS2 Image mesajına çevir ve yayınla
        if frame is not None:
            img_msg = self.br.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(img_msg)
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
