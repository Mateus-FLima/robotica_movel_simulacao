#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import TransformStamped

# Parâmetros de calibração da câmera obtidos do camera_info
# Matriz K da câmera
CAMERA_MATRIX = np.array([
    [381.36246688113556, 0.0, 320.5],
    [0.0, 381.36246688113556, 240.5],
    [0.0, 0.0, 1.0]
])

# Coeficientes de distorção da câmera
DISTORTION_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Tamanho físico da tag
ARUCO_MARKER_SIZE = 0.21

# Dicionário dos marcadores
ARUCO_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)


class ArucoLocator:
    def __init__(self):
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('aruco_locator')

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Buffer e listener para as transformações TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publicador para a pose (opcional, TF é geralmente melhor)
        self.pose_publisher = rospy.Publisher('/robot_pose_aruco', PoseWithCovarianceStamped, queue_size=10)

        # Assina o tópico da imagem da câmera
        self.image_subscriber = rospy.Subscriber(
            '/p3dx/camera/image_raw',
            Image,
            self.image_callback
        )

        rospy.loginfo("Nó localizador Aruco iniciado")

    def image_callback(self, msg):
        try:
            # Converte a mensagem de imagem do ROS para OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detecta os marcadores ArUco na imagem
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                ARUCO_DICTIONARY,
                parameters=cv2.aruco.DetectorParameters_create()
            )

            # Se algum marcador for detectado
            if ids is not None:
                # Estima a pose de cada marcador detectado
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    ARUCO_MARKER_SIZE,
                    CAMERA_MATRIX,
                    DISTORTION_COEFFS
                )

                for i, marker_id in enumerate(ids):
                    # rvec e tvec dão a pose do marcador em relação à câmera
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]

                    marker_frame = f"aruco_marker_{marker_id[0]}"

                    # Publicar a transformação no TF
                    self.publish_transform(
                        tvec, rvec, marker_frame, "camera_link", msg.header.stamp
                    )

                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, CAMERA_MATRIX, DISTORTION_COEFFS, rvec, tvec, 0.1)

                self.print_robot_pose(msg.header.stamp)

            # Mostra a imagem com os marcadores detectados
            cv2.imshow("Deteccao ArUco", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Erro no callback da imagem: {e}")

    def publish_transform(self, tvec, rvec, child_frame, parent_frame, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # Posição (vetor de translação)
        t.transform.translation.x = tvec[0]
        t.transform.translation.y = tvec[1]
        t.transform.translation.z = tvec[2]

        # Orientação (vetor de rotação)
        rotation_matrix = cv2.Rodrigues(rvec)[0]
        quaternion = tf_trans.quaternion_from_matrix(
            np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1]))
        )
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

    def print_robot_pose(self, stamp):
        try:
            # Tenta pegar transformação com timestamp da imagem (mais preciso)
            trans = self.tf_buffer.lookup_transform('map', 'base_link', stamp, rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logwarn("Falha no timestamp exato, usando tempo mais recente")
            try:
                # Se falhar, pega a transformação mais recente disponível
                trans = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
            except Exception as e:
                rospy.logwarn(f"Não foi possível obter transformação: {e}")
                return

        pose = PoseWithCovarianceStamped()
        pose.header = trans.header
        pose.pose.pose.position = trans.transform.translation
        pose.pose.pose.orientation = trans.transform.rotation
        pose.pose.covariance = [0.01] * 36  # Pequena incerteza

        #rospy.loginfo("Pose do robô no frame 'map':")
        #rospy.loginfo(
        #    f" Posição: x={pose.pose.pose.position.x:.2f}, y={pose.pose.pose.position.y:.2f}, z={pose.pose.pose.position.z:.2f}"
        #)
        #rospy.loginfo(
        #    f" Orientação: x={pose.pose.pose.orientation.x:.2f}, y={pose.pose.pose.orientation.y:.2f}, "
        #    f"z={pose.pose.pose.orientation.z:.2f}, w={pose.pose.pose.orientation.w:.2f}"
        #)
        self.pose_publisher.publish(pose)


if __name__ == '__main__':
    try:
        ArucoLocator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
