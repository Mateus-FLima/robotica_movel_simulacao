#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import csv
import os
from datetime import datetime

class PoseLoggerSim:
    def __init__(self):
        self.aruco_pose = None
        self.ekf_pose = None
        self.amcl_pose = None
        self.gazebo_pose = None

        rospy.loginfo("Inicializando PoseLoggerSim...")

        # Pega diretório atual
        cwd = os.getcwd()
        # Volta uma pasta (diretório pai)
        parent_dir = os.path.dirname(cwd)
        # Define a pasta pose_logs dentro do diretório pai
        self.output_folder = os.path.join(parent_dir, "pose_logs")
        os.makedirs(self.output_folder, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_file = os.path.join(self.output_folder, f"pose_log_{timestamp}.csv")

        # Cria cabeçalho do CSV
        with open(self.csv_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "fonte", "x", "y", "z", "yaw"])

        # Subscribers
        rospy.Subscriber("/robot_pose_aruco", PoseWithCovarianceStamped, self.aruco_cb)
        rospy.Subscriber("/ekf_pose", PoseWithCovarianceStamped, self.ekf_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)
        rospy.Subscriber("/p3dx/odom", Odometry, self.gazebo_cb)

        # Timer de impressão e salvamento
        rospy.Timer(rospy.Duration(1.0), self.print_dados)

    def gazebo_cb(self, msg):
        self.gazebo_pose = msg.pose.pose

    def aruco_cb(self, msg):
        self.aruco_pose = msg

    def ekf_cb(self, msg):
        self.ekf_pose = msg

    def amcl_cb(self, msg):
        self.amcl_pose = msg

    def log_to_file(self, label, pose):
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
                yaw = euler[2]

                now = rospy.get_time()

                with open(self.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([now, label, pos.x, pos.y, pos.z, yaw])

            except Exception as e:
                rospy.logerr(f"Erro ao salvar dados de {label}: {e}")

    def print_pose(self, label, pose):
        if pose:
            try:
                if isinstance(pose, PoseWithCovarianceStamped):
                    pos = pose.pose.pose.position
                    ori = pose.pose.pose.orientation
                else:
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

        # Também salva os dados em arquivo
        self.log_to_file("Gazebo", self.gazebo_pose)
        self.log_to_file("EKF", self.ekf_pose)
        self.log_to_file("AMCL", self.amcl_pose)
        self.log_to_file("Aruco", self.aruco_pose)

if __name__ == "__main__":
    rospy.init_node("log_posicao_sim", anonymous=True)
    logger = PoseLoggerSim()
    rospy.spin()
