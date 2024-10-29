import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2
import cv2
import numpy as np
import os
from datetime import datetime

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', 10)

        self.picam2 = Picamera2()
        camera_config = self.picam2.create_video_configuration(
            main={"size": (640, 480)}, 
            controls={"FrameDurationLimits": (33333, 33333)} 
        )
        self.picam2.configure(camera_config)

        self.br = CvBridge()

        self.picam2.start()

        self.distortion_params = np.array([0.262383, -0.953104, -0.005358, 0.002628, 1.16331])
        self.camera_matrix = np.array([[517.306, 0, 318.643],
                                       [0, 516.469, 255.314],
                                       [0, 0, 1]])

        self.declare_parameter('save_images', False)  
        self.declare_parameter('save_folder', '/path/to/save/images')  
        self.save_images = self.get_parameter('save_images').value
        self.save_folder = self.get_parameter('save_folder').value
        self.image_count = 0

        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

        self.timestamp_file = os.path.join(self.save_folder, 'timestamps.txt')

        self.timer = self.create_timer(0.033, self.timer_callback)  

    def timer_callback(self):
        frame = self.picam2.capture_array()

        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        frame = cv2.undistort(frame, self.camera_matrix, self.distortion_params)

        if frame is not None:
            img_msg = self.br.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(img_msg)
            self.get_logger().info('Publishing video frame')

            if self.save_images:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                img_path = os.path.join(self.save_folder, f'{timestamp}.png')
                cv2.imwrite(img_path, frame)
                self.get_logger().info(f'Saved image: {img_path}')
                
                with open(self.timestamp_file, 'a') as f:
                    f.write(f'{timestamp}\n')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
