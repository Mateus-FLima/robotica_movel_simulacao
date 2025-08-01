#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist, PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class PotentialFieldsNode:
    def __init__(self):
        rospy.init_node('potential_fields_node')

        # Parâmetros do YAML
        self.k_att = rospy.get_param('~k_att', 1.0)
        self.k_rep = rospy.get_param('~k_rep', 0.8)
        self.d0 = rospy.get_param('~d0', 1.0)
        self.max_v = rospy.get_param('~max_vel_x', 0.3)
        self.max_w = rospy.get_param('~max_rot_z', 1.0)

        self.goal = None
        self.pose = np.zeros(2)
        self.yaw = 0.0

        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.15)

        rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.scan_cb)
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_cb)
        rospy.Subscriber('/potential_fields/goal', PoseStamped, self.goal_cb)
        self.marker_pub = rospy.Publisher('/campo_forcas', Marker, queue_size=10)
        self.cmd_pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)

    def goal_cb(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])

    def pose_cb(self, msg):
        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def publicar_seta(self, origem, vetor, id, cor, nome):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = nome
        m.id = id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.05  # espessura
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = cor  # RGBA
        m.points = []

        # origem
        p0 = Point()
        p0.x, p0.y = origem[0], origem[1]
        m.points.append(p0)

        # ponta da seta (origem + vetor)
        p1 = Point()
        p1.x = origem[0] + vetor[0]
        p1.y = origem[1] + vetor[1]
        m.points.append(p1)

        self.marker_pub.publish(m)

    def scan_cb(self, msg):
        if self.goal is None:
            return

            # Parar se já está dentro da tolerância
        if np.linalg.norm(self.goal - self.pose) < self.goal_tolerance:
            twist = Twist()
            self.cmd_pub.publish(twist)  # publica velocidade zero
            return

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = np.inf  # lidar com zeros

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.vstack((xs, ys)).T

        # Força atrativa
        F_att = self.k_att * (self.goal - self.pose)

        # Força repulsiva
        F_rep = np.zeros(2)
        for p in points:
            dist = np.linalg.norm(p)
            if 0.01 < dist < self.d0:
                direction = -p / dist
                F_rep += self.k_rep * (1.0 / dist - 1.0 / self.d0) * (1.0 / (dist ** 2)) * direction

        F = F_att + F_rep
        angle_to_goal = np.arctan2(F[1], F[0])
        v = min(self.max_v, np.linalg.norm(F))
        w = angle_to_goal - self.yaw
        w = np.clip(w, -self.max_w, self.max_w)

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)
        self.publicar_seta(self.pose, F_att, 1, (0.0, 1.0, 0.0, 1.0), "atracao")  # verde
        self.publicar_seta(self.pose, F_rep, 2, (1.0, 0.0, 0.0, 1.0), "repulsao")  # vermelho
        self.publicar_seta(self.pose, F, 3, (0.0, 0.0, 1.0, 1.0), "total")  # azul


if __name__ == '__main__':
    PotentialFieldsNode()
    rospy.spin()
