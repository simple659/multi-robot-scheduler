"""
scheduler_node.py
─────────────────────────────────────────────────────────────────
调度节点（SchedulerNode）

【在整个系统中的位置】
  这是系统的"大脑"，负责：
    1. 持有任务队列，按优先级排序
    2. 实时掌握所有机器人的位置和状态
       → 数据来源：订阅每台机器人的 /robot_X/status 话题
    3. 每 3 秒触发一次调度
       → 找出空闲机器人，从队列取任务，做冲突检测
    4. 把分配结果发布到 /robot_X/goal 话题
       → 机器人收到后自己去导航，调度器不管怎么走

【与机器人节点的关系】
  调度器 ←── /robot_X/status ──── 机器人（上报位置+状态）
  调度器 ───→ /robot_X/goal  ───→ 机器人（接收目标坐标）
  两个节点互不知道对方的代码，只通过话题消息交流。

【话题一览】
  订阅：
    /robot_a/status  (std_msgs/String, JSON)
    /robot_b/status  (std_msgs/String, JSON)
    /robot_c/status  (std_msgs/String, JSON)
  发布：
    /robot_a/goal    (geometry_msgs/PoseStamped)
    /robot_b/goal    (geometry_msgs/PoseStamped)
    /robot_c/goal    (geometry_msgs/PoseStamped)
    /scheduler/log   (std_msgs/String, JSON)  供调试用
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import random
import time
from collections import deque


HIGH, MID, LOW = 1, 2, 3
TASK_TYPES = ["搬运货物", "区域巡检", "设备点检", "物料补给"]


class SchedulerNode(Node):

    ROBOT_IDS = ["robot_a", "robot_b", "robot_c"]
    MAX_RESOLVE_ROUNDS = 5

    def __init__(self):
        super().__init__('scheduler_node')
        self.get_logger().info("━" * 50)
        self.get_logger().info("  调度节点启动 [scheduler_node]")
        self.get_logger().info("━" * 50)

        # ── 任务队列 ──
        self.task_queue: deque = deque()
        self.completed_count = 0
        self.failed_count = 0
        self._task_counter = 0

        # ── 机器人状态表 ──
        # 初始状态由这里设定，之后全靠 /status 话题实时刷新
        self.robot_states = {
            "robot_a": {"position": {"x": 0.0, "y": 0.0}, "state": "idle",
                        "battery": 100, "current_task": None},
            "robot_b": {"position": {"x": 9.0, "y": 0.0}, "state": "idle",
                        "battery": 100, "current_task": None},
            "robot_c": {"position": {"x": 4.0, "y": 9.0}, "state": "idle",
                        "battery": 100, "current_task": None},
        }

        # ── 为每台机器人创建 goal Publisher ──
        # 消息类型用 PoseStamped：这是 Nav2 标准的目标位姿格式
        # 机器人节点收到后直接喂给 Nav2 的 NavigateToPose action
        self.goal_pubs = {}
        for rid in self.ROBOT_IDS:
            topic = f"/{rid}/goal"
            self.goal_pubs[rid] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"  [Publisher ← PoseStamped] {topic}")

        # ── 订阅每台机器人的 status 话题 ──
        for rid in self.ROBOT_IDS:
            topic = f"/{rid}/status"
            self.create_subscription(
                String, topic,
                lambda msg, r=rid: self._on_robot_status(r, msg),
                10
            )
            self.get_logger().info(f"  [Subscriber ← String/JSON] {topic}")

        # ── 调度日志话题（方便 ros2 topic echo /scheduler/log 查看） ──
        self.log_pub = self.create_publisher(String, "/scheduler/log", 10)

        # ── 生成初始任务（含一个故意制造的目标冲突，用于演示） ──
        self._generate_tasks(n=8, inject_conflict=True)

        # ── 定时器：每 3 秒触发一次调度循环 ──
        self.sched_timer = self.create_timer(3.0, self._scheduling_loop)
        self.get_logger().info(
            f"\n✓ 初始化完成，任务队列共 {len(self.task_queue)} 个，"
            f"等待第一轮调度...\n"
        )

    # ════════════════════════════════════════════════════════
    # 核心：调度循环（每 3 秒由 Timer 驱动）
    # ════════════════════════════════════════════════════════

    def _scheduling_loop(self):
        """
        每轮调度流程：
          ① 找出所有状态为 idle 的机器人
          ② 从任务队列取出对应数量的任务
          ③ 初始分配（最近机器人优先）
          ④ 冲突检测 → 消解（最多 5 轮）
          ⑤ 下发：发布 PoseStamped 到 /robot_X/goal
        """
        idle = [r for r, s in self.robot_states.items() if s["state"] == "idle"]

        if not self.task_queue and not any(
            s["state"] != "idle" for s in self.robot_states.values()
        ):
            self.get_logger().info("✓ 全部任务已完成，调度器停止。")
            self._print_summary()
            self.sched_timer.cancel()
            return

        if not idle or not self.task_queue:
            return

        n = min(len(idle), len(self.task_queue))
        tasks = [self.task_queue.popleft() for _ in range(n)]
        robots = idle[:n]

        self.get_logger().info(
            f"\n{'─'*50}\n"
            f"  调度触发 | 空闲: {len(idle)} 台 | 本批: {n} 个任务\n"
            f"{'─'*50}"
        )

        assignments = self._initial_assign(robots, tasks)
        self._log_plan("初始分配", assignments)

        assignments = self._resolve_conflicts(assignments)

        self._dispatch(assignments)

    # ════════════════════════════════════════════════════════
    # 话题回调：接收机器人状态
    # ════════════════════════════════════════════════════════

    def _on_robot_status(self, robot_id: str, msg: String):
        """
        机器人每秒上报一次状态。
        这里更新本地状态表，调度器下次触发时用最新数据做分配。
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        prev_state = self.robot_states[robot_id]["state"]
        self.robot_states[robot_id]["position"] = data.get("position",
                                                   self.robot_states[robot_id]["position"])
        self.robot_states[robot_id]["state"]    = data.get("state", prev_state)
        self.robot_states[robot_id]["battery"]  = data.get("battery", 100)

        # 机器人刚完成任务（从非 idle 变成 idle）
        if prev_state != "idle" and data.get("state") == "idle":
            tid = data.get("last_task_id", "?")
            ok  = data.get("success", True)
            pos = data.get("position", {})

            if ok:
                self.completed_count += 1
                self.get_logger().info(
                    f"  ← [{robot_id}] ✓ 完成 {tid} | "
                    f"位置: ({pos.get('x',0):.1f}, {pos.get('y',0):.1f}) | "
                    f"电量: {data.get('battery',0)}%"
                )
            else:
                self.failed_count += 1
                self.get_logger().warn(f"  ← [{robot_id}] ✗ 失败 {tid}，重新入队")
                # 失败任务退回队列头部
                failed = data.get("failed_task")
                if failed:
                    failed["status"] = "pending"
                    self.task_queue.appendleft(failed)

    # ════════════════════════════════════════════════════════
    # 分配 & 冲突检测
    # ════════════════════════════════════════════════════════

    def _initial_assign(self, robot_ids: list, tasks: list) -> dict:
        """最近机器人优先分配（曼哈顿距离）"""
        result = {}
        available = list(robot_ids)
        for task in sorted(tasks, key=lambda t: t["priority"]):
            if not available:
                break
            nearest = min(
                available,
                key=lambda r: (
                    abs(self.robot_states[r]["position"]["x"] - task["target"]["x"]) +
                    abs(self.robot_states[r]["position"]["y"] - task["target"]["y"])
                )
            )
            result[nearest] = task
            available.remove(nearest)
        return result

    def _resolve_conflicts(self, assignments: dict) -> dict:
        """最多 MAX_RESOLVE_ROUNDS 轮冲突消解"""
        for rnd in range(1, self.MAX_RESOLVE_ROUNDS + 1):
            conflicts = self._detect(assignments)
            if not conflicts:
                flag = "无冲突，直接下发" if rnd == 1 else f"第 {rnd-1} 轮消解后通过"
                self.get_logger().info(f"  ✓ {flag}")
                return assignments

            self.get_logger().warn(
                f"  ⚠ 第 {rnd} 轮：检测到 {len(conflicts)} 个冲突"
            )
            for c in conflicts:
                self.get_logger().warn(
                    f"    [{c['severity']}] {c['type']} | "
                    f"{c['robots'][0]} vs {c['robots'][1]} | 位置:{c['location']}"
                )
                if c["type"] == "目标冲突":
                    # 退回优先级低的任务
                    loser = c["robots"][1]
                    if loser in assignments:
                        t = assignments.pop(loser)
                        t["status"] = "pending"
                        self.task_queue.appendleft(t)
                        self.get_logger().info(
                            f"    ↩ {loser} 的 {t['id']} 退回队列"
                        )
                # 路径交叉：简单策略——后一个机器人等一轮（下次调度自动处理）

        self.get_logger().error(
            f"  ✗ 超过 {self.MAX_RESOLVE_ROUNDS} 轮，剩余冲突移交人工"
        )
        return assignments

    def _detect(self, assignments: dict) -> list:
        """检测目标冲突和路径交叉"""
        conflicts = []
        ids = list(assignments.keys())
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                ra, rb = ids[i], ids[j]
                ta, tb = assignments[ra], assignments[rb]

                # 目标冲突
                if ta["target"] == tb["target"]:
                    # 优先级高（数字小）的排前面，保留它
                    ordered = (ra, rb) if ta["priority"] <= tb["priority"] else (rb, ra)
                    conflicts.append({
                        "type": "目标冲突", "severity": "高",
                        "robots": list(ordered),
                        "location": ta["target"],
                    })
                else:
                    # 路径交叉
                    pa = self.robot_states[ra]["position"]
                    pb = self.robot_states[rb]["position"]
                    cross = self._path_cross(
                        [pa["x"], pa["y"]], [ta["target"]["x"], ta["target"]["y"]],
                        [pb["x"], pb["y"]], [tb["target"]["x"], tb["target"]["y"]],
                    )
                    if cross:
                        conflicts.append({
                            "type": "路径交叉", "severity": "中",
                            "robots": [ra, rb],
                            "location": cross,
                        })
        return conflicts

    def _path_cross(self, s1, e1, s2, e2):
        """曼哈顿路径（先横后纵）交叉点检测"""
        def pts(s, e):
            res, x, y = [], int(s[0]), int(s[1])
            sx = 1 if int(e[0]) > x else -1
            while x != int(e[0]):
                x += sx
                res.append((x, y))
            sy = 1 if int(e[1]) > y else -1
            while y != int(e[1]):
                y += sy
                res.append((x, y))
            return res
        inter = set(pts(s1, e1)[:-1]) & set(pts(s2, e2)[:-1])
        return list(inter)[0] if inter else None

    # ════════════════════════════════════════════════════════
    # 下发：发布 PoseStamped 到 /robot_X/goal
    # ════════════════════════════════════════════════════════

    def _dispatch(self, assignments: dict):
        """
        把分配结果发布出去。

        消息类型选用 geometry_msgs/PoseStamped 而不是 String，
        原因：这是 Nav2 NavigateToPose action 直接接受的标准格式，
        机器人节点不需要做任何转换就能喂给导航栈。

        frame_id 设为 'map'，告诉 Nav2 目标坐标是在地图坐标系下。
        """
        self.get_logger().info(f"\n  下发任务 → 发布 PoseStamped 话题：")
        for rid, task in assignments.items():
            # 标记机器人为 executing，防止下轮重复分配
            self.robot_states[rid]["state"] = "executing"
            self.robot_states[rid]["current_task"] = task["id"]

            # 构造 PoseStamped 消息
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"          # 坐标系：地图坐标系
            pose.pose.position.x = float(task["target"]["x"])
            pose.pose.position.y = float(task["target"]["y"])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0         # 朝向：默认正向

            self.goal_pubs[rid].publish(pose)

            # 同时把完整任务信息发到日志话题（方便 echo 调试）
            log = String()
            log.data = json.dumps({
                "event": "task_dispatched",
                "robot": rid,
                "task_id": task["id"],
                "task_type": task["type"],
                "target": task["target"],
                "priority": task["priority"],
                "timestamp": time.time(),
            })
            self.log_pub.publish(log)

            pos = self.robot_states[rid]["position"]
            self.get_logger().info(
                f"  → [{rid}] {task['id']} {task['type']} | "
                f"({pos['x']:.0f},{pos['y']:.0f}) → "
                f"({task['target']['x']:.0f},{task['target']['y']:.0f}) | "
                f"优先级: {'高' if task['priority']==1 else '中' if task['priority']==2 else '低'}"
            )

    # ════════════════════════════════════════════════════════
    # 工具
    # ════════════════════════════════════════════════════════

    def _generate_tasks(self, n: int, inject_conflict: bool):
        random.seed(42)
        tasks = []
        for _ in range(n):
            self._task_counter += 1
            tasks.append({
                "id": f"T{self._task_counter:03d}",
                "target": {
                    "x": float(random.randint(0, 9)),
                    "y": float(random.randint(0, 9)),
                },
                "priority": random.choice([HIGH, HIGH, MID, MID, MID, LOW]),
                "type": random.choice(TASK_TYPES),
                "status": "pending",
            })
        if inject_conflict and len(tasks) >= 5:
            tasks[-1]["target"] = dict(tasks[2]["target"])  # 故意制造目标冲突

        for t in sorted(tasks, key=lambda x: x["priority"]):
            self.task_queue.append(t)
        self.get_logger().info(f"生成 {n} 个任务（含1个故意冲突用于演示）")

    def _log_plan(self, title: str, assignments: dict):
        self.get_logger().info(f"\n  [{title}]")
        for rid, task in assignments.items():
            p = self.robot_states[rid]["position"]
            d = (abs(p["x"] - task["target"]["x"]) +
                 abs(p["y"] - task["target"]["y"]))
            self.get_logger().info(
                f"    {rid} ({p['x']:.0f},{p['y']:.0f}) → "
                f"{task['id']} {task['type']} "
                f"({task['target']['x']:.0f},{task['target']['y']:.0f}) 距离:{d:.0f}"
            )

    def _print_summary(self):
        self.get_logger().info("\n" + "═" * 50)
        self.get_logger().info(
            f"  调度汇总 | ✓ 完成: {self.completed_count} | ✗ 失败: {self.failed_count}"
        )
        for rid, s in self.robot_states.items():
            p = s["position"]
            self.get_logger().info(
                f"  {rid} | 位置:({p['x']:.1f},{p['y']:.1f}) | 电量:{s['battery']}%"
            )
        self.get_logger().info("═" * 50)


def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
