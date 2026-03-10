"""
robot_node.py
─────────────────────────────────────────────────────────────────
机器人节点（RobotNode）

【在整个系统中的位置】
  每台机器人对应一个独立的 ROS2 节点（独立进程）。
  它只做两件事：
    1. 收到调度器发来的目标坐标 → 调用 Nav2 导航过去
    2. 导航完成或失败 → 把结果发布到 /status 话题

【真实性体现在哪里】
  机器人的移动不再是 time.sleep() 假装，而是：
    → 调用 nav2_simple_commander.BasicNavigator.goToPose()
    → Nav2 计算路径，驱动 Gazebo 里的仿真小车真正移动
    → 等待 Nav2 返回 TaskResult（成功/失败/取消）
  位置数据也不再是手动维护，而是：
    → 用 tf2_ros 监听 /tf 变换树
    → 从 odom → base_link 变换中拿到机器人真实坐标

【话题】
  订阅：
    /{robot_id}/goal   (geometry_msgs/PoseStamped)  ← 收目标
  发布：
    /{robot_id}/status (std_msgs/String, JSON)       → 上报状态

【启动参数】
  robot_id   : 机器人名称，如 robot_a（必须和 namespace 一致）
  start_x    : 初始 X 坐标（仅用于首次上报，之后从 TF 读）
  start_y    : 初始 Y 坐标
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — 必须 import 才能注册 TF 变换

import json
import threading
import time


class RobotNode(Node):

    def __init__(self):
        super().__init__('robot_node')

        # ── 从 ROS2 参数服务器读取启动参数 ──
        self.declare_parameter('robot_id', 'robot_a')
        self.declare_parameter('start_x',  0.0)
        self.declare_parameter('start_y',  0.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.position = {
            "x": self.get_parameter('start_x').value,
            "y": self.get_parameter('start_y').value,
        }
        self.state    = 'idle'
        self.battery  = 100
        self.last_task_id = None
        self._nav_lock = threading.Lock()

        self.get_logger().info(
            f"机器人节点启动 [{self.robot_id}] | "
            f"初始位置: ({self.position['x']}, {self.position['y']})"
        )

        # ── TF 监听器：从变换树获取机器人真实坐标 ──
        # Gazebo + Nav2 会持续发布 /tf，我们从中读 odom → base_link
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Nav2 Action Client：向 NavigateToPose action server 发送目标 ──
        # NavigateToPose 是 Nav2 的标准导航接口
        # action server 在 nav2_bringup 启动时自动启动，
        # 名称格式：/{robot_namespace}/navigate_to_pose
        action_name = f"/{self.robot_id}/navigate_to_pose"
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(f"  [ActionClient] {action_name}")

        # 等待 Nav2 action server 上线（最多 10 秒）
        self.get_logger().info("  等待 Nav2 action server...")
        if self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("  ✓ Nav2 已就绪")
        else:
            self.get_logger().warn("  ⚠ Nav2 未就绪，导航将在收到任务后重试连接")

        # ── 订阅调度器发来的目标话题 ──
        goal_topic = f"/{self.robot_id}/goal"
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.get_logger().info(f"  [Subscriber ← PoseStamped] {goal_topic}")

        # ── 发布状态话题 ──
        status_topic = f"/{self.robot_id}/status"
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.get_logger().info(f"  [Publisher → String/JSON] {status_topic}")

        # ── 心跳：每 1 秒上报一次状态 ──
        self.create_timer(1.0, self._heartbeat)

    # ════════════════════════════════════════════════════════
    # 接收任务
    # ════════════════════════════════════════════════════════

    def _on_goal(self, pose: PoseStamped):
        """
        收到调度器发来的 PoseStamped 目标。
        在独立线程中执行导航，不阻塞 ROS2 消息循环。
        """
        if not self._nav_lock.acquire(blocking=False):
            self.get_logger().warn(
                f"  [{self.robot_id}] 正在执行任务，忽略新目标"
            )
            return

        # 把 PoseStamped 里的坐标存下来，任务完成时用
        target = {
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
        }
        self.get_logger().info(
            f"  ← [{self.robot_id}] 收到目标: ({target['x']:.1f}, {target['y']:.1f})"
        )
        self.state = 'navigating'

        # 独立线程执行
        t = threading.Thread(
            target=self._navigate,
            args=(pose, target),
            daemon=True
        )
        t.start()

    # ════════════════════════════════════════════════════════
    # 真实导航（Nav2 Action）
    # ════════════════════════════════════════════════════════

    def _navigate(self, pose: PoseStamped, target: dict):
        """
        调用 Nav2 NavigateToPose action，让机器人真实移动到目标位置。

        Nav2 内部做的事：
          1. AMCL 定位：确认机器人当前在地图上的位置
          2. Global Planner：规划从当前位置到目标的全局路径（A* 等）
          3. Local Planner：沿路径行走，实时避障（DWA 等）
          4. 到达目标后返回 TaskResult.SUCCEEDED
        """
        try:
            # 构造 action goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            goal_msg.behavior_tree = ''  # 使用 Nav2 默认行为树

            # 异步发送 goal，拿到 future
            send_goal_future = self._nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self._on_nav_feedback
            )

            # 等待 action server 接受 goal（rclpy 的 spin_until_future_complete）
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().error("  Nav2 拒绝了目标，任务失败")
                self._finish(success=False, target=target, task=None)
                return

            self.get_logger().info(
                f"  [{self.robot_id}] Nav2 接受目标，开始导航..."
            )
            self.state = 'navigating'

            # 等待导航完成
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            from nav2_msgs.action import NavigateToPose as NTP
            if result.status == 4:  # GoalStatus.STATUS_SUCCEEDED = 4
                self.get_logger().info(
                    f"  [{self.robot_id}] ✓ 导航成功，到达 "
                    f"({target['x']:.1f}, {target['y']:.1f})"
                )
                self._finish(success=True, target=target, task=None)
            else:
                self.get_logger().warn(
                    f"  [{self.robot_id}] ✗ 导航失败，状态码: {result.status}"
                )
                self._finish(success=False, target=target, task=None)

        except Exception as e:
            self.get_logger().error(f"  导航异常: {e}")
            self._finish(success=False, target=target, task=None)
        finally:
            self._nav_lock.release()

    def _on_nav_feedback(self, feedback_msg):
        """
        Nav2 导航过程中的实时反馈回调。
        feedback 包含：当前位置、剩余距离、已用时间等。
        """
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().info(
            f"  [{self.robot_id}] 导航中... 剩余距离: {dist:.2f} m",
            throttle_duration_sec=2.0  # 每 2 秒打印一次，避免刷屏
        )

    # ════════════════════════════════════════════════════════
    # 任务完成处理
    # ════════════════════════════════════════════════════════

    def _finish(self, success: bool, target: dict, task):
        """
        导航完成（成功或失败）后：
          1. 从 TF 读取真实位置更新 self.position
          2. 扣除电量
          3. 发布状态话题通知调度器
        """
        # 从 TF 变换树获取机器人真实坐标
        real_pos = self._get_position_from_tf()
        if real_pos:
            self.position = real_pos
        elif success:
            self.position = target  # TF 读取失败时退而求其次用目标坐标

        # 扣电量（距离越远耗电越多）
        if success:
            dist = (abs(self.position.get("x", 0) - target["x"]) +
                    abs(self.position.get("y", 0) - target["y"]))
            self.battery = max(0, self.battery - int(dist * 2))

        self.state = 'idle'
        self._publish_status(success=success, task=task)

    def _get_position_from_tf(self) -> dict:
        """
        从 TF 变换树读取机器人真实位置。

        TF 是 ROS2 的坐标变换系统，Gazebo 仿真时会持续发布：
          map → odom → base_link 这条变换链

        我们查询 map → base_link 就能得到机器人在地图中的坐标。
        """
        try:
            # lookup_transform(目标坐标系, 源坐标系, 时间)
            trans = self.tf_buffer.lookup_transform(
                "map",
                f"{self.robot_id}/base_link",  # Gazebo 里每台机器人有自己的 namespace
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            return {
                "x": round(trans.transform.translation.x, 2),
                "y": round(trans.transform.translation.y, 2),
            }
        except Exception:
            return None  # 获取失败时返回 None，调用方自己处理

    # ════════════════════════════════════════════════════════
    # 状态上报
    # ════════════════════════════════════════════════════════

    def _heartbeat(self):
        """每秒定时上报（心跳），让调度器知道自己还活着"""
        # 同时尝试刷新一次位置
        real = self._get_position_from_tf()
        if real:
            self.position = real
        self._publish_status()

    def _publish_status(self, success: bool = True, task=None):
        """发布状态到 /{robot_id}/status"""
        payload = {
            "robot_id":    self.robot_id,
            "position":    self.position,
            "state":       self.state,
            "battery":     self.battery,
            "last_task_id": self.last_task_id,
            "success":     success,
            "timestamp":   time.time(),
        }
        if task and not success:
            payload["failed_task"] = task

        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
