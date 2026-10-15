#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import json
from std_msgs.msg import String

from pathlib import Path

class MissionPlannerNode(Node):

    def __init__(self) -> None:

        #下面这个函数的功能是：创建并初始化一个名为 mission_planner 的ROS 2节点
        super().__init__("mission_planner")

        #在终端会打印出来"Mission planner node started"
        self.get_logger().info(
            "Mission planner node started"
        )

        # ——————————————————————————————————————————————————————初始化一些变量
        # 保存progress的地址初始化
        self.progress_file = Path(
            r"D:\a_autowork\TJROBOTX\Final\主状态机\log\progress.jsonl"
        )

        # 创建log文件夹
        self.progress_file.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        # 创建progress.jsonl文件
        self.progress_file.touch(
            exist_ok=True,
        )

        # 初始化主状态
        self.state = "WAIT_FOR_BEGIN"

        # 先初始化UAV和USV的progress
        self.uav_progress = None
        self.usv_progress = None

        # ——————————————————————————————————————————————————————创建消息的subscriber
        # 订阅UAV progress
        self.uav_progress_sub = self.create_subscription(
            String,
            "/tongji_mrl/tsky/progress",
            self._on_uav_progress,
            10,
        )

        # 订阅USV progress
        self.usv_progress_sub = self.create_subscription(
            String,
            "/tongji_mrl/twave/progress",
            self._on_usv_progress,
            10,
        )

        # 订阅比赛开始的信息
        self.run_start_sub = self.create_subscription(
            String,
            "/tongji_mrl/mission/run_start",
            self._on_run_start,
            10,
        )

        # ——————————————————————————————————————————————————————创建消息的publisher
        # 创建给ocs发布progress消息的publisher
        self.mission_status_pub = self.create_publisher(
            String,
            "/tongji_mrl/mission/status",
            10,
        )

        # 给UAV发布任务命令
        self.uav_task_cmd_pub = self.create_publisher(
            String,
            "/tongji_mrl/tsky/task_cmd",
            10,
        )

        # 给USV发布任务命令
        self.usv_task_cmd_pub = self.create_publisher(
            String,
            "/tongji_mrl/twave/task_cmd",
            10,
        )

    #————————————————————————————————————————————————————————————————————————————————————————————————定义回调函数
    # 回调函数：一接收uav progress的回调函数
    def _on_uav_progress(self, msg: String) -> None:
        try:
            self.uav_progress = json.loads(msg.data)
            self.publish_combined_progress()
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Received invalid UAV progress"
            )

    # 回调函数：一接收usv progress的回调函数
    def _on_usv_progress(self, msg: String) -> None:
        try:
            self.usv_progress = json.loads(msg.data)
            self.publish_combined_progress()
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Received invalid USV progress"
            )

    # 回调函数：一接收到"/tongji_mrl/mission/run_start"的比赛开始的信息，就且换状态为MAP_EXECUTE；且会调用函数，向uav和usv发布开始map任务的信息
    def _on_run_start(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Received invalid run_start message"
            )
            return

        # 确保消息内容是字典
        if not isinstance(payload, dict):
            self.get_logger().warning(
                "run_start message must be a JSON object"
            )
            return

        # 只有WAIT_FOR_BEGIN状态可以接收begin
        if self.state != "WAIT_FOR_BEGIN":
            return

        # 检查是否为开始命令
        if payload.get("command") != "begin":
            return

        # 切换到MAP_EXECUTE
        self.set_main_state("MAP_EXECUTE")

        # 通知UAV和USV开始MAP任务
        self.publish_map_start()

    #————————————————————————————————————————————————————————————————————————————————————————————————定义其它的辅助函数
    # 整合uav和usv，以及本节点的状态，来发布的消息，进行写入本地的记录文件中，然后调用progress_publisher发布出去
    def publish_combined_progress(self) -> None:
        payload = {
            "mission_planner": self.mission_planner_progress,
            "uav": self.uav_progress,
            "usv": self.usv_progress,
        }

        # 把payload追加写入jsonl文件
        with self.progress_file.open(
            "a",
            encoding="utf-8",
        ) as file:
            file.write(
                json.dumps(
                    payload,
                    ensure_ascii=False,
                )
                + "\n"
            )

        msg = String()
        msg.data = json.dumps(
            payload,
            ensure_ascii=False,
        )
        #self.mission_status_pub是初始化里有创建过的发布函数
        self.mission_status_pub.publish(msg)

    # 切换主节点状态，并且打印log
    def set_main_state(self, new_state: str) -> None:
        old_state = self.state
        self.state = new_state

        self.get_logger().info(
            f"Main state changed: "
            f"{old_state} -> {new_state}"
        )

        #更新完之后，同时也需要publish combined progress
        self.publish_combined_progress()

    # 当接收到task begin的信息时候，其回调函数_on_run_start会调用这个函数，用于给uav和usv，比赛开始的信息
    def publish_map_start(self) -> None:
        # UAV任务命令
        uav_payload = {
            "vehicle": "uav",
            "task": "MAP",
            "progress": None,
            "error": None,
            "command": "begin",
        }

        uav_msg = String()
        uav_msg.data = json.dumps(
            uav_payload,
            ensure_ascii=False,
        )

        self.uav_task_cmd_pub.publish(uav_msg)

        # USV任务命令
        usv_payload = {
            "vehicle": "usv",
            "task": "MAP",
            "progress": None,
            "error": None,
            "command": "begin",
        }

        usv_msg = String()
        usv_msg.data = json.dumps(
            usv_payload,
            ensure_ascii=False,
        )

        self.usv_task_cmd_pub.publish(usv_msg)

def main(args=None) -> None:

    #初始化ROS 2的Python运行环境/开启ros2功能
    rclpy.init(args=args)

    #创建这个node，创建的时候会自动执行MissionPlannerNode.__init__()
    node = MissionPlannerNode()

    try:
        #让节点持续运行
        rclpy.spin(node)
    #允许键盘停止,例如当ctrl+c的时候就会停止
    except KeyboardInterrupt:
        pass
    #释放节点和关闭ROS
    finally:
        #关闭这个节点，释放其占用的资源
        node.destroy_node()
        #关闭整个python ros2运行环境
        rclpy.shutdown()


if __name__ == "__main__":
    main()