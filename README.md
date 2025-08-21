```markdown
# Car Controller

这是一个用于比赛的机器人小车控制器项目，基于 ROS (Robot Operating System) 开发。

## 项目概述

`car_controller` 是一个我为了参加国赛，在国赛期间为我手上的一个ROS小车开发的控制组件
也就是说这个项目是不完整的，光有本项目没用，还要搭配那个小车本身的系统才行

## 功能特性

- **指令系统**: 通过指令控制小车行为
- **多端控制**: 写了串口和tcp服务器的控制连接接口，随便改，原本是和我写的服务器和语音控制模块通信

## 安装说明

### 环境要求

- Ubuntu 20.04 LTS
- ROS Noetic
- Python 3.8+

### 安装步骤

1. 克隆项目到ROS工作空间:
```bash
cd ~
mkdir rnm_ws/src
cd rnm_ws/src
git clone https://github.com/YunshiZean/car_controller.git
```

2. 编译项目:
```bash
cd ~/rnm_ws
catkin_make
```

3. 源环境设置:
```bash
#注意，这里要在那个车的系统下，bashrc里面是还有别的东西的哦
source devel/setup.bash --extend
```

## 使用方法

### 启动控制器

```bash
roslaunch car_controller car_controller.launch
```

### 节点信息

- `central_manager`: 中央管理器
- `serial_exchange`: 串口交换节点
- `tcp_exchange`: tcp交换器

## 项目结构

```
car_controller/
├── src/                 # C源代码目录（空的）
├── launch/              # 启动文件
├── config/              # 配置文件
├── scripts/             # 脚本文件
└── CMakeLists.txt       # 构建配置
```

## 配置说明

配置文件位于 `config/` 目录下，主要包括:
- `waypoints.json`: 路径点注册
- `paths.json`: 路径注册

## 贡献指南

欢迎提交 Issue 和 Pull Request 来改进这个项目。
虽然这个项目太简单和简陋，还不全，估计没什么人会看到，除了内部人员访问，但是欢迎提issue和pr。

## 许可证

不熟，随便用，用了说一声

## 联系方式
email: yunshistar1314520@163.com
QQ: 2410978942
```