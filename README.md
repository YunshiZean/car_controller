```markdown
# Car Controller

这是一个用于比赛的机器人小车控制器项目，基于 ROS (Robot Operating System) 开发。

## 项目概述

`car_controller` 是一个专为机器人小车比赛设计的控制器包，实现了小车的运动控制、传感器数据处理、路径规划等核心功能。

## 功能特性

- **运动控制**: 实现小车的前进、后退、转向等基本运动控制
- **传感器集成**: 集成各类传感器数据处理
- **路径规划**: 基本的路径规划和避障功能
- **ROS集成**: 完全基于ROS架构，便于扩展和维护

## 安装说明

### 环境要求

- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+

### 安装步骤

1. 克隆项目到ROS工作空间:
```bash
cd ~/ros_ws/src
git clone <repository-url> car_controller
```

2. 编译项目:
```bash
cd ~/ros_ws
catkin_make
```

3. 源环境设置:
```bash
source devel/setup.bash
```

## 使用方法

### 启动控制器

```bash
roslaunch car_controller car_controller.launch
```

### 节点信息

- `car_controller_node`: 主控制器节点
- `sensor_processor`: 传感器数据处理节点
- `motion_planner`: 运动规划节点

## 项目结构

```
car_controller/
├── src/                 # 源代码目录
├── launch/              # 启动文件
├── config/              # 配置文件
├── scripts/             # 脚本文件
└── CMakeLists.txt       # 构建配置
```

## 配置说明

配置文件位于 `config/` 目录下，主要包括:
- `car_params.yaml`: 小车参数配置
- `controller_params.yaml`: 控制器参数配置

## 贡献指南

欢迎提交 Issue 和 Pull Request 来改进这个项目。

## 许可证

MIT License

## 联系方式

如有问题，请联系项目维护者。
```