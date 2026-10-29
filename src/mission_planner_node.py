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
        self.progress_file = (
            Path.home()
            / ".ros"
            / "robotx_mission_state_machine"
            / "progress.jsonl"
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

        # 记录是否已经收到run_start
        self.run_start_received = False

        # 记录是否已经收到config
        self.course_config_received = False

        # 记录Task1开始命令是否已经发送
        self.task1_start_sent = False

        # 记录Task3开始命令是否已经发送
        self.task3_start_sent = False

        # Task4相关信息
        self.task4_target_vehicle = None
        self.task4_command_seq = None
        self.task4_position = None

        # UAV、USV进入Task4之前的任务断点
        self.uav_checkpoint = None
        self.usv_checkpoint = None

        # 防止重复发送Task4报告和恢复命令
        self.task4_ack_sent = False
        self.task4_ready_sent = False
        self.task4_resume_sent = False

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

        # 接收OCS发送的赛场配置信息
        self.course_config_sub = self.create_subscription(
            String,
            "/tongji_mrl/mission/course_config",
            self._on_course_config,
            10,
        )

        # 订阅USV任务结果
        self.usv_result_sub = self.create_subscription(
            String,
            "/tongji_mrl/twave/result",
            self._on_usv_result,
            10,
        )

        # 接收OCS发送的Task4命令
        self.task4_cmd_sub = self.create_subscription(
            String,
            "/tongji_mrl/mission/task4_cmd",
            self._on_task4_command,
            10,
        )

        # 接收UAV进入Task4前的checkpoint
        self.uav_checkpoint_sub = self.create_subscription(
            String,
            "/tongji_mrl/tsky/checkpoint",
            self._on_uav_checkpoint,
            10,
        )

        # 接收USV进入Task4前的checkpoint
        self.usv_checkpoint_sub = self.create_subscription(
            String,
            "/tongji_mrl/twave/checkpoint",
            self._on_usv_checkpoint,
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

        # 向UAV转发赛场配置信息
        self.uav_course_config_pub = self.create_publisher(
            String,
            "/tongji_mrl/tsky/course_config",
            10,
        )

        # 向OCS发布任务报告
        self.mission_task_report_pub = self.create_publisher(
            String,
            "/tongji_mrl/mission/task_report",
            10,
        )

    #————————————————————————————————————————————————————————————————————————————————————————————————定义回调函数
    # 回调函数：一接收uav progress的回调函数
    def _on_uav_progress(self, msg: String) -> None:
        try:
            self.uav_progress = json.loads(msg.data)

            # 如果发现错误，进入FAILED并停止后续判断
            if self.check_vehicle_error():
                return

            self.publish_combined_progress()
            self.check_map_finished()
            self.check_task1_finished()
            self.check_task3_finished()
            self.check_task4_progress()
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Received invalid UAV progress"
            )

    # 回调函数：一接收usv progress的回调函数
    def _on_usv_progress(self, msg: String) -> None:
        try:
            self.usv_progress = json.loads(msg.data)

            # 如果发现错误，进入FAILED并停止后续判断
            if self.check_vehicle_error():
                return
            
            self.publish_combined_progress()
            self.check_map_finished()
            self.check_task1_finished()
            self.check_task3_finished()
            self.check_task4_progress()
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
        
        # 记录已经收到run_start
        self.run_start_received = True

        self.get_logger().info(
            "Run start received"
        )

        # 检查是否同时已经收到course_config
        self.try_start_map()

    def _on_course_config(self, msg: String) -> None:
        # 记录已经收到course_config
        self.course_config_received = True

        # 把赛场配置信息转发给UAV
        forwarded_msg = String()
        forwarded_msg.data = msg.data

        self.uav_course_config_pub.publish(
            forwarded_msg
        )

        self.get_logger().info(
            "Course configuration received and forwarded to UAV"
        )

        # 检查是否同时已经收到run_start
        self.try_start_map()

    # 收到USV任务结果后，原样转发给OCS
    def _on_usv_result(self, msg: String) -> None:
        report_msg = String()
        report_msg.data = msg.data

        self.mission_task_report_pub.publish(
            report_msg
        )

        self.get_logger().info(
            "USV task result forwarded to OCS"
        )

    # 接收OCS发送的Task4命令
    def _on_task4_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Received invalid Task4 command"
            )
            return

        if not isinstance(payload, dict):
            return

        if payload.get("task") != "TASK4":
            return

        command = payload.get("command")

        if not isinstance(command, dict):
            return

        command_type = command.get("type")

        if command_type == "ASSISTANCE_REQUEST":
            self.handle_assistance_request(
                payload,
                command,
            )

        elif command_type == "READINESS_CONFIRM":
            self.handle_readiness_confirm(
                payload,
                command,
            )

    # 接收UAV在进入Task4前保存的任务断点
    def _on_uav_checkpoint(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Received invalid UAV checkpoint"
            )
            return

        if not isinstance(payload, dict):
            return

        if payload.get("vehicle") != "uav":
            return

        if payload.get("command") != "checkpoint":
            return

        self.uav_checkpoint = payload

        self.get_logger().info(
            "UAV checkpoint received"
        )

        self.try_resume_previous_tasks()


    # 接收USV在进入Task4前保存的任务断点
    def _on_usv_checkpoint(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                "Received invalid USV checkpoint"
            )
            return

        if not isinstance(payload, dict):
            return

        if payload.get("vehicle") != "usv":
            return

        if payload.get("command") != "checkpoint":
            return

        self.usv_checkpoint = payload

        self.get_logger().info(
            "USV checkpoint received"
        )

        self.try_resume_previous_tasks()

    #————————————————————————————————————————————————————————————————————————————————————————————————定义其它的辅助函数
    # 整合uav和usv，以及本节点的状态，来发布的消息，进行写入本地的记录文件中，然后调用progress_publisher发布出去
    def publish_combined_progress(self) -> None:
        payload = {
            "mission_planner": self.state,
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

    # 同时收到run_start和course_config后开始MAP
    def try_start_map(self) -> None:
        # 只能从WAIT_FOR_BEGIN开始
        if self.state != "WAIT_FOR_BEGIN":
            return

        # 必须收到run_start
        if not self.run_start_received:
            return

        # 必须收到course_config
        if not self.course_config_received:
            return

        self.get_logger().info(
            "Run start and course configuration received; starting MAP"
        )

        self.set_main_state("MAP_EXECUTE")
        self.publish_map_start()

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

    # MAP完成检查函数
    def check_map_finished(self) -> None:
        # 已经发送过Task1命令，不再继续!!这里来确保task1的发送指令只有一次，因为如果已经是True了，就直接返回，不会往下进行到发布命令了
        if self.task1_start_sent:
            return

        # 必须处于MAP_EXECUTE
        if self.state != "MAP_EXECUTE":
            return

        # 必须已经收到双方progress
        if not isinstance(self.uav_progress, dict):
            return

        if not isinstance(self.usv_progress, dict):
            return

        uav_finished = (
            self.uav_progress.get("task") == "MAP"
            and self.uav_progress.get("progress") == "MAP_FINISH"
            and self.uav_progress.get("error") is None
        )

        usv_finished = (
            self.usv_progress.get("task") == "MAP"
            and self.usv_progress.get("progress") == "MAP_FINISH"
            and self.usv_progress.get("error") is None
        )

        if uav_finished and usv_finished:
            # 先记录已经发送，避免重复发送
            self.task1_start_sent = True

            # 切换主状态
            self.set_main_state("TASK1_EXECUTE")

            # 向双方发送Task1开始命令
            self.publish_task1_start()

    # Task1命令发布函数
    def publish_task1_start(self) -> None:
        uav_payload = {
            "vehicle": "uav",
            "task": "TASK1",
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

        usv_payload = {
            "vehicle": "usv",
            "task": "TASK1",
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

        self.get_logger().info(
            "Task1 begin command sent to UAV and USV"
        )

    # Task1完成检查函数
    def check_task1_finished(self) -> None:
        if self.task3_start_sent:
            return

        if self.state != "TASK1_EXECUTE":
            return

        if not isinstance(self.uav_progress, dict):
            return

        if not isinstance(self.usv_progress, dict):
            return

        uav_finished = (
            self.uav_progress.get("task") == "TASK1"
            and self.uav_progress.get("progress") == "TASK1_FINISH"
            and self.uav_progress.get("error") is None
        )

        usv_finished = (
            self.usv_progress.get("task") == "TASK1"
            and self.usv_progress.get("progress") == "TASK1_FINISH"
            and self.usv_progress.get("error") is None
        )

        if uav_finished and usv_finished:
            self.task3_start_sent = True

            self.set_main_state(
                "TASK3_EXECUTE"
            )

            self.publish_task3_start()
    
    # Task3命令发布函数
    def publish_task3_start(self) -> None:
        uav_payload = {
            "vehicle": "uav",
            "task": "TASK3",
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

        usv_payload = {
            "vehicle": "usv",
            "task": "TASK3",
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

        self.get_logger().info(
            "Task3 begin command sent to UAV and USV"
        )

    # Task3完成检查函数
    def check_task3_finished(self) -> None:
        # 必须处于TASK3_EXECUTE状态
        if self.state != "TASK3_EXECUTE":
            return

        # 必须已经收到UAV和USV的progress
        if not isinstance(self.uav_progress, dict):
            return

        if not isinstance(self.usv_progress, dict):
            return

        # 判断UAV是否完成Task3
        uav_finished = (
            self.uav_progress.get("task") == "TASK3"
            and self.uav_progress.get("progress") == "TASK3_FINISH"
            and self.uav_progress.get("error") is None
        )

        # 判断USV是否完成Task3
        usv_finished = (
            self.usv_progress.get("task") == "TASK3"
            and self.usv_progress.get("progress") == "TASK3_FINISH"
            and self.usv_progress.get("error") is None
        )

        # UAV和USV都完成后进入COMPLETE
        if uav_finished and usv_finished:
            self.set_main_state("COMPLETE")

    # 检查UAV或USV是否报告错误
    def check_vehicle_error(self) -> bool:
        # COMPLETE或FAILED状态不再重复处理错误
        if self.state in ("COMPLETE", "FAILED"):
            return self.state == "FAILED"

        failed_vehicle = None
        error_content = None

        # 检查UAV错误
        if isinstance(self.uav_progress, dict):
            uav_error = self.uav_progress.get("error")

            if uav_error is not None:
                failed_vehicle = "uav"
                error_content = uav_error

        # 检查USV错误
        if failed_vehicle is None and isinstance(
            self.usv_progress,
            dict,
        ):
            usv_error = self.usv_progress.get("error")

            if usv_error is not None:
                failed_vehicle = "usv"
                error_content = usv_error

        # 双方都没有错误
        if failed_vehicle is None:
            return False

        self.get_logger().error(
            f"{failed_vehicle.upper()} reported error: "
            f"{error_content}"
        )

        # 主状态切换为FAILED
        self.set_main_state("FAILED")

        # 通知UAV和USV结束任务
        self.publish_task_end(
            failed_vehicle=failed_vehicle,
            error_content=error_content,
        )

        return True

    # 因为载体发生错误，通知UAV和USV结束当前任务
    def publish_task_end(
        self,
        *,
        failed_vehicle: str,
        error_content,
    ) -> None:
        # 获取双方当前执行的任务
        uav_task = None
        usv_task = None

        if isinstance(self.uav_progress, dict):
            uav_task = self.uav_progress.get("task")

        if isinstance(self.usv_progress, dict):
            usv_task = self.usv_progress.get("task")

        error_message = (
            f"{failed_vehicle}: {error_content}"
        )

        # 通知UAV结束任务
        uav_payload = {
            "vehicle": "uav",
            "task": uav_task,
            "progress": None,
            "error": error_message,
            "command": "end",
        }

        uav_msg = String()
        uav_msg.data = json.dumps(
            uav_payload,
            ensure_ascii=False,
        )
        self.uav_task_cmd_pub.publish(uav_msg)

        # 通知USV结束任务
        usv_payload = {
            "vehicle": "usv",
            "task": usv_task,
            "progress": None,
            "error": error_message,
            "command": "end",
        }

        usv_msg = String()
        usv_msg.data = json.dumps(
            usv_payload,
            ensure_ascii=False,
        )
        self.usv_task_cmd_pub.publish(usv_msg)

        self.get_logger().error(
            "Mission failed; end command sent to UAV and USV"
        )

    # AssistanceRequest处理函数
    def handle_assistance_request(
        self,
        payload: dict,
        command: dict,
    ) -> None:
        # Task4只允许打断MAP、Task1和Task3
        if self.state not in (
            "MAP_EXECUTE",
            "TASK1_EXECUTE",
            "TASK3_EXECUTE",
        ):
            return

        target_vehicle = payload.get("vehicle")
        command_seq = command.get("command_seq")
        position = command.get("position")

        if target_vehicle not in ("uav", "usv"):
            self.get_logger().warning(
                "Invalid Task4 target vehicle"
            )
            return

        if command_seq is None:
            self.get_logger().warning(
                "Task4 command_seq is missing"
            )
            return

        if not isinstance(position, dict):
            self.get_logger().warning(
                "Task4 position is missing"
            )
            return

        # 保存Task4命令内容
        self.task4_target_vehicle = target_vehicle
        self.task4_command_seq = command_seq
        self.task4_position = position

        # 清除上一次Task4的数据
        self.uav_checkpoint = None
        self.usv_checkpoint = None
        self.task4_ack_sent = False
        self.task4_ready_sent = False
        self.task4_resume_sent = False

        # 进入Task4
        self.set_main_state("TASK4_EXECUTE")

        # 通知UAV和USV进入Task4
        self.publish_task4_start()

    # Task4命令发布函数
    def publish_task4_start(self) -> None:
        uav_payload = {
            "vehicle": "uav",
            "task": "TASK4",
            "progress": None,
            "error": None,
            "command": {
                "type": "TASK4_BEGIN",
                "command_seq": self.task4_command_seq,
                "target_vehicle": self.task4_target_vehicle,
                "position": self.task4_position,
            },
        }

        uav_msg = String()
        uav_msg.data = json.dumps(
            uav_payload,
            ensure_ascii=False,
        )
        self.uav_task_cmd_pub.publish(uav_msg)

        usv_payload = {
            "vehicle": "usv",
            "task": "TASK4",
            "progress": None,
            "error": None,
            "command": {
                "type": "TASK4_BEGIN",
                "command_seq": self.task4_command_seq,
                "target_vehicle": self.task4_target_vehicle,
                "position": self.task4_position,
            },
        }

        usv_msg = String()
        usv_msg.data = json.dumps(
            usv_payload,
            ensure_ascii=False,
        )
        self.usv_task_cmd_pub.publish(usv_msg)

        self.get_logger().info(
            "Task4 begin command sent to UAV and USV"
        )
    
    # 检查task4的progress函数
    def check_task4_progress(self) -> None:
        if self.state != "TASK4_EXECUTE":
            return

        # 只检查Task4目标载体的progress
        if self.task4_target_vehicle == "uav":
            target_progress = self.uav_progress

        elif self.task4_target_vehicle == "usv":
            target_progress = self.usv_progress

        else:
            return

        if not isinstance(target_progress, dict):
            return

        if target_progress.get("task") != "TASK4":
            return

        progress = target_progress.get("progress")

        # 目标载体确认收到Task4命令
        if (
            progress == "TASK4_ACK"
            and not self.task4_ack_sent
        ):
            self.task4_ack_sent = True

            self.publish_task4_report(
                report_type="INCIDENT_ACK",
                progress="TASK4_ACK",
            )

        # 目标载体到达位置并准备好
        elif (
            progress == "TASK4_READY"
            and not self.task4_ready_sent
        ):
            self.task4_ready_sent = True

            self.publish_task4_report(
                report_type="READINESS_REPORT",
                progress="TASK4_READY",
            )
    
    # Task4报告发布函数
    def publish_task4_report(
        self,
        *,
        report_type: str,
        progress: str,
    ) -> None:
        payload = {
            "vehicle": self.task4_target_vehicle,
            "task": "TASK4",
            "progress": progress,
            "error": None,
            "command": {
                "type": report_type,
                "command_seq": self.task4_command_seq,
            },
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            ensure_ascii=False,
        )

        self.mission_task_report_pub.publish(msg)

        self.get_logger().info(
            f"Task4 report sent: {report_type}"
        )

    # ReadinessConfirm处理函数
    def handle_readiness_confirm(
        self,
        payload: dict,
        command: dict,
    ) -> None:
        if self.state != "TASK4_EXECUTE":
            return

        if payload.get("vehicle") != self.task4_target_vehicle:
            return

        report_seq = command.get("report_seq")

        if report_seq is None:
            self.get_logger().warning(
                "ReadinessConfirm report_seq is missing"
            )
            return

        self.get_logger().info(
            f"ReadinessConfirm received: {report_seq}"
        )

        # 进入Task4结束与恢复阶段
        self.set_main_state("TASK4_OVER")

        self.try_resume_previous_tasks()
    
    # 恢复函数
    def try_resume_previous_tasks(self) -> None:
        if self.state != "TASK4_OVER":
            return

        if self.task4_resume_sent:
            return

        # 必须已经收到双方checkpoint
        if not isinstance(self.uav_checkpoint, dict):
            return

        if not isinstance(self.usv_checkpoint, dict):
            return

        uav_task = self.uav_checkpoint.get("task")
        usv_task = self.usv_checkpoint.get("task")

        # 正常情况下双方应该处于相同主任务
        if uav_task != usv_task:
            self.get_logger().warning(
                "UAV and USV checkpoint tasks do not match"
            )
            return

        resume_states = {
            "MAP": "MAP_EXECUTE",
            "TASK1": "TASK1_EXECUTE",
            "TASK3": "TASK3_EXECUTE",
        }

        resume_state = resume_states.get(uav_task)

        if resume_state is None:
            self.get_logger().warning(
                f"Unsupported checkpoint task: {uav_task}"
            )
            return

        self.task4_resume_sent = True

        # 恢复UAV
        uav_payload = {
            "vehicle": "uav",
            "task": uav_task,
            "progress": self.uav_checkpoint.get("progress"),
            "error": None,
            "command": "resume",
        }

        uav_msg = String()
        uav_msg.data = json.dumps(
            uav_payload,
            ensure_ascii=False,
        )
        self.uav_task_cmd_pub.publish(uav_msg)

        # 恢复USV
        usv_payload = {
            "vehicle": "usv",
            "task": usv_task,
            "progress": self.usv_checkpoint.get("progress"),
            "error": None,
            "command": "resume",
        }

        usv_msg = String()
        usv_msg.data = json.dumps(
            usv_payload,
            ensure_ascii=False,
        )
        self.usv_task_cmd_pub.publish(usv_msg)

        # 恢复原来的主状态
        self.set_main_state(resume_state)

        self.get_logger().info(
            f"Previous mission resumed: {resume_state}"
        )

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
