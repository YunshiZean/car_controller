#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

A = PoseStamped()
B = PoseStamped()
C = PoseStamped()
D = PoseStamped()
E = PoseStamped()
F = PoseStamped()
G = PoseStamped()
H = PoseStamped()

if __name__ == '__main__':
    rospy.init_node('path_publisher')

    A.header.frame_id = "map"
    A.header.stamp = rospy.Time.now()
    A.pose.position.x = 0.177
    A.pose.position.y = -0.056
    A.pose.orientation.z = 0.0301
    A.pose.orientation.w = 0.9994

    B.header.frame_id = "map"
    B.header.stamp = rospy.Time.now()
    B.pose.position.x = 1.418
    B.pose.position.y = -0.097
    B.pose.orientation.z = -0.017
    B.pose.orientation.w = 0.9999

    C.header.frame_id = "map"
    C.header.stamp = rospy.Time.now()
    C.pose.position.x = 1.581
    C.pose.position.y = -0.134
    C.pose.orientation.z = -0.2309
    C.pose.orientation.w = 0.9732

    D.header.frame_id = "map"
    D.header.stamp = rospy.Time.now()
    D.pose.position.x = 1.816
    D.pose.position.y = -0.285
    D.pose.orientation.z = -0.271
    D.pose.orientation.w = 0.9631

    E.header.frame_id = "map"
    E.header.stamp = rospy.Time.now()
    E.pose.position.x = 2.00
    E.pose.position.y = -0.55
    E.pose.orientation.z = -0.669
    E.pose.orientation.w = 0.7426

    F.header.frame_id = "map"
    F.header.stamp = rospy.Time.now()
    F.pose.position.x = 1.986
    F.pose.position.y = -1.119
    F.pose.orientation.z = -0.723
    F.pose.orientation.w = 0.695

    G.header.frame_id = "map"
    G.header.stamp = rospy.Time.now()
    G.pose.position.x = 1.861
    G.pose.position.y = -1.385
    G.pose.orientation.z = 0.938
    G.pose.orientation.w = -0.346

    H.header.frame_id = "map"
    H.header.stamp = rospy.Time.now()
    H.pose.position.x = 1.694
    H.pose.position.y = -1.514
    H.pose.orientation.z = 0.949
    H.pose.orientation.w = -0.316








    path_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/via_points', Path, queue_size=1)
    path = Path()

    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()
    path.poses.append(A)
    path.poses.append(B)
    path.poses.append(C)
    path.poses.append(D)
    path.poses.append(E)
    path.poses.append(F)
    path.poses.append(G)
    path.poses.append(H)
    path_pub.publish(path)
    rospy.loginfo("OK master. Task is done.")

    # while not rospy.is_shutdown():
    #     path.header.stamp = rospy.Time.now()
    #     path.header.frame_id = 'map'
    #     path.poses.append(PoseStamped())
    #     path_pub.publish(path)
    #     rospy.sleep(1)