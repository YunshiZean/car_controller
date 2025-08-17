#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped

if __name__ == "__main__":
    rospy.init_node("test")
    pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=10)

    rospy.sleep(1)  # 等待一下，确保连接建立

    p = PointStamped()
    p.header.frame_id = "map"
    p.header.stamp = rospy.Time.now()
    p.point.x = 5.7274513
    p.point.y = 1.582512
    p.point.z = 0

    rospy.logwarn("publishing...")
    pub.publish(p)

    rospy.spin()  # 保持节点活着，不让消息丢掉
