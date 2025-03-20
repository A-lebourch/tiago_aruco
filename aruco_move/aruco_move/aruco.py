import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.img_subscription = self.create_subscription(Image, '/head_front_camera/image', self.image_callback, 10)
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose', self.amcl_pose_callback, 10)
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

        # Publisher pour commander le robot
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'key_vel', qos)

        # Matrice de calibration de la caméra (à ajuster selon ton setup)
        self.camera_matrix = np.array([[800, 0, 320], 
                                       [0, 800, 240], 
                                       [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((4, 1))  # Si pas de distorsion
        
        self.navigator = BasicNavigator()

        self.facing_aruco = False
        self.aruco_detected = False
        self.current_pose = None
        self.stop = False

    def amcl_pose_callback(self, msg):
        if not self.facing_aruco :
            self.current_pose = msg.pose.pose
            print(f"current pose : [{self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, {self.current_pose.orientation.w:.2f}]")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                self.aruco_detected = True  # Un marqueur est détecté, arrêter la rotation
                aruco.drawDetectedMarkers(cv_image, corners, ids)

                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 1, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                    x, y, z = tvecs[i][0]
                    rx, ry, rz = self.get_euler_angles(rvecs[i])

                    if not self.facing_aruco:
                    #     self.reloc(x, y, z, rx, ry, rz)
                    #     self.stop = True
                    # elif self.facing_aruco :
                        self.goto_aruco(x, y, z, rx, ry, rz)
                        self.facing_aruco = True
                        self.search_for_aruco()
            else:
                if not self.aruco_detected:
                    self.search_for_aruco()

            cv2.imshow("Aruco Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur lors du traitement de l'image : {e}")

    def get_euler_angles(self, rvec):
        R, _ = cv2.Rodrigues(rvec)

        # Extraire les angles d’Euler en suivant la convention ZYX (yaw, pitch, roll)
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)

        singular = sy < 1e-6

        if not singular:
            rx = np.arctan2(R[2, 1], R[2, 2])  # Rotation autour de X (roll)
            ry = np.arctan2(-R[2, 0], sy)       # Rotation autour de Y (pitch)
            rz = np.arctan2(R[1, 0], R[0, 0])  # Rotation autour de Z (yaw)
        else:
            rx = np.arctan2(-R[1, 2], R[1, 1])
            ry = np.arctan2(-R[2, 0], sy)
            rz = 0

        # Convertir en degrés
        rx, ry, rz = np.degrees([rx, ry, rz])

        return rx, ry, rz

    def reloc(self, x, y, z, rx, ry, rz):
        """
        Commande le robot pour se placer à 4m du marqueur et face à lui.
        
        Args:
            x, y, z: Position du marqueur ArUco en mètres.
            rx, ry, rz: Rotation du marqueur en degrés.
        """
        twist = Twist()
        delta_x = z - 4.0
        delta_z = -x
        k_lin = 0.1 
        k_ang = 0.5
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        if z < 4.5:
            self.facing_aruco = True
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = float(delta_x * k_lin)
            twist.angular.z = float(delta_z * k_ang)

        self.pub.publish(twist)

    def goto_aruco(self, x, y, z, rx, ry, rz):

        init_pose = self.current_pose
        if init_pose != None:

            self.navigator.waitUntilNav2Active()
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = init_pose.position.x - x if rx > 0 else init_pose.position.x + x
            goal_pose.pose.position.y = init_pose.position.y - z + 5.5
            goal_pose.pose.orientation = init_pose.orientation
            self.navigator.goToPose(goal_pose)
        print(rx)
        # print(f"[{init_pose.position.x:.2f}, {init_pose.position.y:.2f}, {init_pose.orientation.w:.2f}]")



    def search_for_aruco(self):
        """            goal_pose.pose.orientation.w = 0.0

        Fait tourner le robot sur lui-même jusqu'à ce qu'un marqueur ArUco soit détecté.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3  # Tourne lentement sur lui-même

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
