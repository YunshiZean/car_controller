# coding=utf-8

"""
文件名: WayManager.py
简介：寻路管理模块
作者： 未定义实验室.Zean 罗灵轩
版本： 1.1
说明： 修改了点位对象的信息
创建时间： 2025.8.9
最后更新时间： 2025.8.12
"""

from collections import deque
from typing import List, Optional



class Point:
    def __init__(self, name: str, out_list: List["Point"]):
        self.name = name
        self.out_list = out_list

    def __repr__(self):
        return f"Point({self.name})"


# ====== 你的图（保持不变）======
A = Point('A',[])
B = Point('B',[])
C = Point('C',[])
D = Point('D',[])
E = Point('E',[])
F = Point('F',[])
G = Point('G',[])
H = Point('H',[])
I = Point('I',[])
J = Point('J',[])
K = Point('K',[])
L = Point('L',[])
M = Point('M',[])
N = Point('N',[])
O = Point('O',[])
P = Point('P',[])
Q = Point('Q',[])
R = Point('R',[])
S = Point('S',[])
T = Point('T',[])






A.out_list = [I,B]
B.out_list = [C]
C.out_list = [D, R]
D.out_list = [E]
E.out_list = [F, O]
F.out_list = [G]
G.out_list = [H, L]
H.out_list = [A]
I.out_list = [J]
J.out_list = [K, M, O]
K.out_list = [H]
L.out_list = [M,Q,S]
M.out_list = [N]
N.out_list = [F]
O.out_list = [P]
P.out_list = [Q,S,K]
Q.out_list = [D]
R.out_list = [S,K,M]
S.out_list = [T]
T.out_list = [B]


# =================================

from typing import Dict, Optional
def bfs_shortest_path(start: Point, goal: Point) -> Optional[List[Point]]:
    parent: Dict[Point, Optional[Point]] = {start: None}
    """返回从 start 到 goal 的最短路径（按边数），不可达则返回 None。"""
    if start is goal:
        return [start]

    q = deque([start])
    visited = {start}
    # 用于回溯路径：child -> parent
    parent = {start: None}

    while q:
        u = q.popleft()
        for v in u.out_list:
            if v not in visited:
                visited.add(v)
                parent[v] = u
                if v is goal:
                    # 回溯构造路径
                    path = [v]
                    while parent[path[-1]] is not None:
                        path.append(parent[path[-1]])
                    path.reverse()
                    return path
                q.append(v)

    return None


def find_path(start: Point, goal: Point):
    """
    寻找从 start 到 goal 的最短路径
    返回最短路径的Point列表，如果不存在则返回 None
    """
    _path = bfs_shortest_path(start, goal)
    if _path is None:
        return None
    return [p.name.__str__() for p in _path]

def to_point(name: str):
    """
    将 Point 的 name 转换为 Point 对象
    """
    if name == A.name:
        return A
    elif name == B.name:
        return B
    elif name == C.name:
        return C
    elif name == D.name:
        return D
    elif name == E.name:
        return E
    elif name == F.name:
        return F
    elif name == G.name:
        return G
    elif name == H.name:
        return H
    elif name == I.name:
        return I
    elif name == J.name:
        return J
    elif name == K.name:
        return K
    elif name == L.name:
        return L
    elif name == M.name:
        return M
    elif name == N.name:
        return N
    elif name == O.name:
        return O
    elif name == P.name:
        return P
    elif name == Q.name:
        return Q
    elif name == R.name:
        return R
    elif name == S.name:
        return S
    elif name == T.name:
        return T




# ==== 示例 ====
if __name__ == "__main__":
    print(find_path(to_point("A"), to_point("A")))
    # path = bfs_shortest_path(A, H)
    # if path is None:
    #     print("A -> H 不可达")
    # else:
    #     print("最短路径：", " -> ".join(p.name for p in path), f"(步数 {len(path)-1})")

# 你也可以随便换起点终点，比如：
# print(bfs_shortest_path(H, M))
