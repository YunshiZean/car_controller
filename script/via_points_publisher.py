#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
import actionlib
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

# ===================== 这里填你的锚点（地图坐标，单位：米） =====================
# 例如：[(x1,y1), (x2,y2), ...]，建议顺时针/逆时针依次排列；默认闭环
ANCHORS = [
    (0.177, -0.056),
    (1.418, -0.097),
    (1.581, -0.134),
    (1.816, -0.285),
    (2.00, -0.55),
    (1.986,-1.119),
    (1.861,-1.385),
    (1.694,-1.514)
]
# ============================================================================

def centripetal_catmull_rom(p0, p1, p2, p3, n=20, alpha=0.5):
    """从 p1->p2 段生成 n+1 个样条点（含端点），向心 Catmull-Rom，稳定不易过冲"""
    def tj(ti, pa, pb):
        dx, dy = pb[0]-pa[0], pb[1]-pa[1]
        return (dx*dx + dy*dy) ** (alpha*0.5) + ti

    t0 = 0.0
    t1 = tj(t0, p0, p1)
    t2 = tj(t1, p1, p2)
    t3 = tj(t2, p2, p3)

    ts = [t1 + (t2 - t1) * i / float(n) for i in range(n + 1)]
    out = []
    for t in ts:
        # 一次插值
        A1 = ((t1 - t) / (t1 - t0) * p0[0] + (t - t0) / (t1 - t0) * p1[0],
              (t1 - t) / (t1 - t0) * p0[1] + (t - t0) / (t1 - t0) * p1[1])
        A2 = ((t2 - t) / (t2 - t1) * p1[0] + (t - t1) / (t2 - t1) * p2[0],
              (t2 - t) / (t2 - t1) * p1[1] + (t - t1) / (t2 - t1) * p2[1])
        A3 = ((t3 - t) / (t3 - t2) * p2[0] + (t - t2) / (t3 - t2) * p3[0],
              (t3 - t) / (t3 - t2) * p2[1] + (t - t2) / (t3 - t2) * p3[1])
        # 二次插值
        B1 = ((t2 - t) / (t2 - t0) * A1[0] + (t - t0) / (t2 - t0) * A2[0],
              (t2 - t) / (t2 - t0) * A1[1] + (t - t0) / (t2 - t0) * A2[1])
        B2 = ((t3 - t) / (t3 - t1) * A2[0] + (t - t1) / (t3 - t1) * A3[0],
              (t3 - t) / (t3 - t1) * A2[1] + (t - t1) / (t3 - t1) * A3[1])
        # 三次插值（最终点）
        C = ((t2 - t) / (t2 - t1) * B1[0] + (t - t1) / (t2 - t1) * B2[0],
             (t2 - t) / (t2 - t1) * B1[1] + (t - t1) / (t2 - t1) * B2[1])
        out.append(C)
    return out

def build_spline_points(anchors, step=0.15, closed=True, samples_per_seg=20, alpha=0.5):
    """将锚点拟合为平滑曲线，并按弧长等距重采样（间距 step）"""
    m = len(anchors)
    assert m >= 3, "至少需要 3 个锚点"

    # 1) 生成粗样条点
    rough = []
    if closed:
        rng = range(m)
    else:
        rng = range(m - 1)  # 开环：最后一段到倒数第二个点
    for i in rng:
        # 端点延拓（开环时）
        p0 = anchors[(i - 1) % m] if closed or i > 0 else anchors[0]
        p1 = anchors[i]
        p2 = anchors[(i + 1) % m] if closed or i + 1 < m else anchors[-1]
        p3 = anchors[(i + 2) % m] if closed or i + 2 < m else anchors[-1]
        seg = centripetal_catmull_rom(p0, p1, p2, p3, n=samples_per_seg, alpha=alpha)
        if i > 0:
            seg = seg[1:]  # 去掉段首重复点
        rough += seg

    # 2) 弧长累积
    acc = [0.0]
    for i in range(1, len(rough)):
        dx = rough[i][0] - rough[i-1][0]
        dy = rough[i][1] - rough[i-1][1]
        acc.append(acc[-1] + math.hypot(dx, dy))
    total = acc[-1]

    # 3) 等距重采样
    n_out = max(4, int(total / step))
    targets = [i * total / n_out for i in range(n_out)]
    res = []
    j = 1
    for s in targets:
        while j < len(acc) and acc[j] < s:
            j += 1
        j = min(j, len(acc) - 1)
        t = 0.0 if acc[j] == acc[j-1] else (s - acc[j-1]) / (acc[j] - acc[j-1])
        x = rough[j-1][0] * (1 - t) + rough[j][0] * t
        y = rough[j-1][1] * (1 - t) + rough[j][1] * t
        res.append((x, y))
    return res

def yaw_of(points, i):
    x1, y1 = points[i]
    x2, y2 = points[(i + 1) % len(points)]
    return math.atan2(y2 - y1, x2 - x1)

def make_path(points, frame):
    """构造 nav_msgs/Path（每个 PoseStamped 都带 frame+stamp）"""
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = rospy.Time.now()
    for i, (x, y) in enumerate(points):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = x
        ps.pose.position.y = y
        yaw = yaw_of(points, i)
        q = quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation = Quaternion(*q)
        path.poses.append(ps)
    return path

class ViaPointsAndCarrot:
    """一次性发布整条 via_points（Path），并持续用“前视点”发 move_base 目标"""
    def __init__(self, points):
        self.points = points
        self.N = len(points)

        # 读取参数
        self.frame = rospy.get_param("~frame_id", "map")
        self.via_topic = rospy.get_param("~via_points_topic",
                                        "/move_base/TebLocalPlannerROS/via_points")
        self.debug_path_topic = rospy.get_param("~debug_path_topic", "/smooth_path")

        self.step = float(rospy.get_param("~step", 0.15))
        self.lookahead = float(rospy.get_param("~lookahead", 1.5))
        self.update_hz = float(rospy.get_param("~update_hz", 2.0))
        self.advance_min = int(rospy.get_param("~advance_min", 2))
        self.publish_posearray_dbg = bool(rospy.get_param("~publish_posearray_debug", False))

        # 发布者
        self.via_pub = rospy.Publisher(self.via_topic, Path, queue_size=1, latch=True)
        self.dbg_pub = rospy.Publisher(self.debug_path_topic, Path, queue_size=1, latch=True)
        self.pa_dbg_pub = rospy.Publisher("/teb_dbg/via_points_pa", PoseArray,
                                            queue_size=1, latch=True) if self.publish_posearray_dbg else None

        # TF & action client
        self.tf = tf.TransformListener()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("move_base 已就绪")

        # 发布一次整条 via-points（Path）
        path = make_path(self.points, self.frame)
        self.via_pub.publish(path)
        self.dbg_pub.publish(path)
        if self.pa_dbg_pub:
            pa = PoseArray(); pa.header.frame_id = self.frame; pa.header.stamp = rospy.Time.now()
            for i,(x,y) in enumerate(self.points):
                pose = Pose()
                pose.position.x = x; pose.position.y = y
                q = quaternion_from_euler(0,0,yaw_of(self.points,i))
                pose.orientation = Quaternion(*q)
                pa.poses.append(pose)
            self.pa_dbg_pub.publish(pa)
        rospy.loginfo("Smooth path & via_points published. points=%d", len(self.points))

        # 定时器：持续刷新“胡萝卜目标”
        self.last_goal_idx = -1
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_hz), self.tick)

    def robot_xy(self):
        self.tf.waitForTransform(self.frame, "base_link", rospy.Time(0), rospy.Duration(0.2))
        (t, _) = self.tf.lookupTransform(self.frame, "base_link", rospy.Time(0))
        return t[0], t[1]

    def nearest_index(self, x, y):
        # 简单最近点搜索（如需更稳，可限定在“上次索引±窗口”）
        best_i, best_d = 0, 1e18
        for i, (px, py) in enumerate(self.points):
            d = (px - x) * (px - x) + (py - y) * (py - y)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def send_goal_idx(self, i):
        x, y = self.points[i]
        yaw = yaw_of(self.points, i)
        q = quaternion_from_euler(0, 0, yaw)

        g = MoveBaseGoal()
        g.target_pose.header.frame_id = self.frame
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        g.target_pose.pose.orientation = Quaternion(*q)

        self.client.send_goal(g)
        rospy.loginfo("→ carrot goal @ idx=%d  (x=%.2f, y=%.2f, yaw=%.2f)", i, x, y, yaw)

    def tick(self, _):
        try:
            rx, ry = self.robot_xy()
        except Exception as e:
            rospy.logwarn("TF lookup failed: %s", repr(e))
            return

        i0 = self.nearest_index(rx, ry)
        k = int(round(self.lookahead / max(self.step, 1e-3)))  # 防零
        i_goal = (i0 + max(1, k)) % self.N

        # 只有当目标沿路径前进了足够步数，才重新抢占，避免无意义频繁 send_goal
        if self.last_goal_idx < 0 or ((i_goal - self.last_goal_idx) % self.N) >= self.advance_min:
            self.send_goal_idx(i_goal)
            self.last_goal_idx = i_goal

def main():
    rospy.init_node("continuous_patrol")

    # 可把 ANCHORS 换成 rosparam（若你想从 yaml 读）
    anchors = rospy.get_param("~anchors", ANCHORS)
    frame = rospy.get_param("~frame_id", "map")
    step = float(rospy.get_param("~step", 0.15))
    closed = bool(rospy.get_param("~closed_loop", True))
    samples_per_seg = int(rospy.get_param("~samples_per_seg", 20))
    alpha = float(rospy.get_param("~alpha", 0.5))

    points = build_spline_points(anchors, step=step, closed=closed,
                                samples_per_seg=samples_per_seg, alpha=alpha)
    ViaPointsAndCarrot(points)
    rospy.spin()

if __name__ == "__main__":
    main()
