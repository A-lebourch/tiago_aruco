import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'key_vel', qos)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=float)
            self.dist_coeffs = np.zeros((4, 1)) 
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)

                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 1, self.camera_matrix, self.dist_coeffs)
                for i in range(len(ids)):
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)
                    
                    x, y, z = tvecs[i][0]
                    cv2.putText(cv_image, f"x: {x:.2f}m y: {y:.2f}m z: {z:.2f}m", 
                                tuple(corners[i][0][0].astype(int)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    delta_x = 0
                    # delta_x = z - 4.0
                    # delta_z = 0.0
                    delta_z = 0.0 - x
                    twist = Twist()
                    k_lin = 1.0 
                    k_ang = 1.0
                    twist.linear.x = delta_x * k_lin
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = delta_z *k_ang

                    self.pub.publish(twist)
                    
                # for i in range(len(ids)):
                #     rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
                #     aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

            cv2.imshow("Aruco Detection", cv_image)
            cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f"Erreur lors du traitement de l'image : {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
