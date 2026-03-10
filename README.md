# 🤖 Multi-Robot Task Scheduler

> 基于 ROS2 Humble + Nav2 + Gazebo 的多机器人任务调度系统  
> 调度器通过话题与机器人节点通信，机器人通过 Nav2 真实导航

---

## 系统架构

```
调度节点 (scheduler_node)
  │
  ├─── 发布 /robot_a/goal (PoseStamped) ──→  机器人节点 A
  ├─── 发布 /robot_b/goal (PoseStamped) ──→  机器人节点 B
  └─── 发布 /robot_c/goal (PoseStamped) ──→  机器人节点 C
                                              │
                                              └── 调用 Nav2 action
                                                  NavigateToPose
                                                  机器人在 Gazebo 里真实移动
                                                  从 TF 读取真实位置
  ↑
  订阅 /robot_X/status (String/JSON)
  ← 机器人完成后上报结果
```

---

## 文件说明

| 文件 | 作用 |
|---|---|
| `scheduler_node.py` | 任务队列管理、冲突检测、分配下发 |
| `robot_node.py` | 接收目标、调用 Nav2 导航、上报状态 |
| `launch/launch.py` | 启动 Gazebo + Nav2 + 所有节点 |
| `package.xml` | 声明 ROS2 包的所有依赖 |
| `setup.py` | 注册节点名称，colcon build 时用 |

---

## Windows 环境配置（WSL2）

### 第一步：安装 WSL2

以管理员身份打开 PowerShell，执行：
```powershell
wsl --install -d Ubuntu-22.04
```
重启电脑后，打开"Ubuntu 22.04"应用完成初始化（设置用户名和密码）。

---

### 第二步：在 WSL2 里安装 ROS2 Humble

打开 Ubuntu 22.04 终端，逐行执行：
```bash
# 添加 ROS2 软件源
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# 安装
sudo apt update
sudo apt install -y ros-humble-desktop
```

---

### 第三步：安装 Gazebo + TurtleBot3 + Nav2

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander

# 设置环境变量（加到 ~/.bashrc 里，每次打开终端自动生效）
echo "source /opt/ros/humble/setup.bash"         >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger"             >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc
```

---

### 第四步：编译本项目

```bash
# 克隆仓库
git clone https://github.com/YOUR_USERNAME/multi-robot-scheduler.git
cd multi-robot-scheduler/ros2_ws

# 安装依赖
rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build

# 加载编译结果
source install/setup.bash
```

---

### 第五步：运行

```bash
ros2 launch multi_robot_scheduler launch.py
```

Gazebo 窗口会弹出，三台小车开始在地图里移动执行任务。

---

### 调试命令

另开终端（同样要先 `source install/setup.bash`）：

```bash
# 查看调度器发出的任务日志
ros2 topic echo /scheduler/log

# 查看 robot_a 收到的目标
ros2 topic echo /robot_a/goal

# 查看 robot_a 的状态上报
ros2 topic echo /robot_a/status

# 查看所有活跃话题
ros2 topic list

# 查看所有节点
ros2 node list
```

---

## License
MIT
