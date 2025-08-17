"""
文件名: WayManager.py
简介： 路径管理
作者： 未定义实验室.Zean 罗灵轩
版本： 2.0
说明： 为了适配osm地图而作出修改
更新内容： 
创建时间： 2025.8.5
最后更新时间： 2025.8.17
"""
import json
import os
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_from_euler

class WaypointManager:
    def __init__(self, base_dir:str):
        with open(os.path.join(base_dir, "waypoints.json"), "r") as f:
            self.points = json.load(f) #载入

        with open(os.path.join(base_dir, "paths.json"), "r") as f:
            self.paths = json.load(f) #载入

    #获取点的值
    def get_pose(self, name:str) ->PointStamped:
        if name not in self.points:
            raise ValueError(f"点位 {name} 不存在")
        p = self.points[name]
        point = PointStamped()
        point.header.frame_id = "map"
        point.point.position.x = p["x"]
        point.point.position.y = p["y"]
        point.point.position.z = 0
        return point

    def get_path(self, path_name:str) ->list:#获取路径的点
        return self.paths.get(path_name, [])

    def get_all_paths(self) ->list:#获取json中全部的路径
        return list(self.paths.keys())

    def get_all_points(self) ->list:#获取json中全部的点
        return list(self.points.keys())


if __name__ == "__main__":
    import os

    test = WaypointManager(os.path.join(os.path.dirname(__file__),"..","config"))
    A = test.get_all_points()
    print(A)
    pass