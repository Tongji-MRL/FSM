"""Persistent, transport-free Task 4 safety gate for MissionPlannerNode."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping


SCHEMA = "twave-mission-task4-v1"
TYPES = {"ASSISTANCE_REQUEST", "READINESS_CONFIRM", "READINESS_REPORT_PUBLISHED", "KEEP_OUT_ZONE", "ALL_CLEAR", "MOVING_OBJECT_ALERT"}


class Task4Guard:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.record: dict[str, Any] = {}
        if self.path.exists():
            try:
                self.record = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                self.record = {"unsafe_restart": True}

    def _save(self) -> None:
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.record, sort_keys=True), encoding="utf-8")
        temporary.replace(self.path)

    def accept(self, payload: Mapping[str, Any], *, active_run_id: int | None, planner_state: str) -> str:
        if payload.get("schema_version") != SCHEMA or payload.get("task") != "TASK4":
            raise ValueError("unsupported Task4 schema or task")
        run_id, command_id, vehicle, command = payload.get("run_id"), payload.get("command_id"), payload.get("vehicle"), payload.get("command")
        if run_id != active_run_id or not isinstance(command_id, str) or not command_id:
            raise ValueError("Task4 run_id or command_id is invalid")
        if vehicle not in {"uav", "usv"} or not isinstance(command, Mapping) or command.get("type") not in TYPES:
            raise ValueError("Task4 vehicle or command is invalid")
        if not isinstance(command.get("command_seq"), int) or command["command_seq"] <= 0:
            raise ValueError("Task4 command_seq is invalid")
        kind = command["type"]
        if command_id == self.record.get("command_id"):
            return "DUPLICATE"
        if kind == "READINESS_REPORT_PUBLISHED":
            if (self.record.get("phase") != "READY_REPORTED" or payload.get("vehicle") != self.record.get("vehicle")
                    or command.get("command_seq") != self.record.get("command_seq")
                    or not isinstance(command.get("report_seq"), int) or command["report_seq"] <= 0
                    or command.get("internal_origin") != "ocs"):
                raise ValueError("invalid OCS ReadinessReport publication notice")
            if self.record.get("readiness_event_id") != command.get("readiness_event_id"):
                raise ValueError("ReadinessReport publication event does not match")
            self.record["readiness_report_seq"] = command["report_seq"]
            self.record["phase"] = "WAITING_READINESS_CONFIRM"
            self._save()
            return "READINESS_REPORT_RECORDED"
        if kind != "ASSISTANCE_REQUEST":
            return "POLICY_NOT_READY"
        position = command.get("position")
        if planner_state not in {"MAP_EXECUTE", "TASK1_EXECUTE", "TASK3_EXECUTE"}:
            raise ValueError("Task4 AssistanceRequest is illegal in current state")
        if not isinstance(position, Mapping) or not all(isinstance(position.get(key), (int, float)) for key in ("latitude", "longitude")):
            raise ValueError("Task4 AssistanceRequest position is invalid")
        self.record = {"run_id": run_id, "command_id": command_id, "vehicle": vehicle,
                       "command_seq": command["command_seq"], "position": dict(position),
                       "pause_acks": {}, "checkpoints": {}, "phase": "WAITING_SAFE_PAUSE"}
        self._save()
        return "ACCEPTED_WAITING_SAFE_PAUSE"

    def pause_ack(self, vehicle: str, *, run_id: int, command_id: str, accepted: bool) -> bool:
        if (self.record.get("phase") != "WAITING_SAFE_PAUSE"
                or run_id != self.record.get("run_id")
                or command_id != self.record.get("command_id")):
            return False
        if vehicle not in {"uav", "usv"} or not accepted:
            return False
        self.record["pause_acks"][vehicle] = True; self._save(); return True

    def checkpoint(self, vehicle: str, value: Mapping[str, Any]) -> bool:
        if self.record.get("phase") != "WAITING_SAFE_PAUSE":
            return False
        if value.get("vehicle") != vehicle or value.get("run_id") != self.record.get("run_id") or value.get("source") != vehicle:
            return False
        if value.get("pause_command_id") != self.record.get("command_id") or not isinstance(value.get("timestamp"), str) or not value.get("task") or "progress" not in value:
            return False
        self.record["checkpoints"][vehicle] = dict(value); self._save(); return True

    def can_start(self) -> bool:
        if self.record.get("phase") != "WAITING_SAFE_PAUSE":
            return False
        if not isinstance(self.record.get("run_id"), int) or not self.record.get("command_id"):
            return False
        acknowledgements = self.record.get("pause_acks", {})
        checkpoints = self.record.get("checkpoints", {})
        if not isinstance(acknowledgements, Mapping) or not isinstance(checkpoints, Mapping):
            return False
        return all(
            acknowledgements.get(vehicle) is True
            and self._valid_checkpoint(vehicle, checkpoints.get(vehicle))
            for vehicle in ("uav", "usv")
        )

    def _valid_checkpoint(self, vehicle: str, value: Any) -> bool:
        """Revalidate persisted checkpoints before crossing the start gate."""
        return bool(
            isinstance(value, Mapping)
            and value.get("vehicle") == vehicle
            and value.get("source") == vehicle
            and value.get("run_id") == self.record.get("run_id")
            and value.get("pause_command_id") == self.record.get("command_id")
            and isinstance(value.get("timestamp"), str)
            and value.get("timestamp")
            and isinstance(value.get("task"), str)
            and value.get("task")
            and "progress" in value
        )

    def mark_started(self) -> None:
        if not self.can_start():
            raise ValueError("Task4 safe pause/checkpoint gate is incomplete")
        self.record["phase"] = "EXECUTING"; self._save()

    def execution_safety_status(self) -> tuple[str, list[str]]:
        """Return status-v2 safety fields only for a persisted execution phase."""
        if self.record.get("phase") != "EXECUTING":
            raise ValueError("Task4 is not in EXECUTING phase")
        # The interrupted mission is safely paused; Task4 itself is active.
        # This is intentionally not READY and has no recovery authorization.
        return "TASK4_EXECUTING", []

    def readiness(self, value: Mapping[str, Any]) -> bool:
        if self.record.get("phase") != "EXECUTING":
            return False
        if value.get("task") != "TASK4" or value.get("progress") != "TASK4_READY" or value.get("command_id") != self.record.get("command_id"):
            return False
        self.record["readiness_event_id"] = f"{self.record['command_id']}:readiness"
        self.record["phase"] = "READY_REPORTED"; self._save(); return True

    def confirm(self, payload: Mapping[str, Any], *, readiness_report_seq: int | None, safety_state: str) -> bool:
        command = payload.get("command", {})
        return bool(self.record.get("phase") == "WAITING_READINESS_CONFIRM" and payload.get("run_id") == self.record.get("run_id")
                    and payload.get("vehicle") == self.record.get("vehicle") and command.get("type") == "READINESS_CONFIRM"
                    and command.get("report_seq") == readiness_report_seq == self.record.get("readiness_report_seq") and command.get("original_command_seq") == self.record.get("command_seq")
                    and safety_state not in {"SAFE_HOLD", "SAFE_STOP"})
