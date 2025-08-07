#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PoseLoggerSim:
    def __init__(self):
        self.aruco_pose = None
        self.ekf_pose = None
        self.amcl_pose = None
        self.gazebo_pose = None  # Vem de /p3dx/odom

        rospy.loginfo("Inicializando PoseLoggerSim...")

        # Sempre cria os Subscribers, mesmo que o tópico ainda não esteja ativo
        rospy.Subscriber("/robot_pose_aruco", PoseWithCovarianceStamped, self.aruco_cb)
        rospy.Subscriber("/ekf_pose", PoseWithCovarianceStamped, self.ekf_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)
        rospy.Subscriber("/p3dx/odom", Odometry, self.gazebo_cb)

        # Timer de impressão
        rospy.Timer(rospy.Duration(1.0), self.print_dados)

    def gazebo_cb(self, msg):
        try:
            self.gazebo_pose = msg.pose.pose
        except Exception as e:
            rospy.logerr(f"Erro no gazebo_cb: {e}")

    def aruco_cb(self, msg):
        try:
            self.aruco_pose = msg
        except Exception as e:
            rospy.logerr(f"Erro no aruco_cb: {e}")

    def ekf_cb(self, msg):
        try:
            self.ekf_pose = msg
        except Exception as e:
            rospy.logerr(f"Erro no ekf_cb: {e}")

    def amcl_cb(self, msg):
        try:
            self.amcl_pose = msg
        except Exception as e:
            rospy.logerr(f"Erro no amcl_cb: {e}")

    def print_pose(self, label, pose):
        if pose:
            try:
                if isinstance(pose, PoseWithCovarianceStamped):
                    pos = pose.pose.pose.position
                    ori = pose.pose.pose.orientation
                else:  # Odometry do Gazebo
                    pos = pose.position
                    ori = pose.orientation

                quat = (ori.x, ori.y, ori.z, ori.w)
                euler = euler_from_quaternion(quat)
                print(f"[{label}] x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f} | yaw: {euler[2]:.2f}")
            except Exception as e:
                rospy.logerr(f"Erro ao imprimir {label}: {e}")
        else:
            print(f"[{label}] Dados não disponíveis.")

    def print_dados(self, event):
        print("\n--- Posições atuais ---")
        self.print_pose("Gazebo", self.gazebo_pose)
        self.print_pose("EKF", self.ekf_pose)
        self.print_pose("AMCL", self.amcl_pose)
        self.print_pose("Aruco", self.aruco_pose)

if __name__ == "__main__":
    rospy.init_node("log_posicao_sim", anonymous=True)
    logger = PoseLoggerSim()
    rospy.spin()
