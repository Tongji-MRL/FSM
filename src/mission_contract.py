"""Pure validation and status helpers for the formal MissionPlannerNode."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
from typing import Any, Mapping


COURSE_SCHEMA = "twave-course-config-v1"
STATUS_SCHEMA = "twave-mission-status-v2"
RUN_SCHEMAS = {"twave-mission-run-v1", "twave-mission-dryrun-v1"}


@dataclass(frozen=True)
class Course:
    course_id: str
    pinger_freq_hz: int
    corners: tuple[tuple[float, float], ...]
    fingerprint: str


def _timestamp(value: Any, name: str, *, nullable: bool = False) -> None:
    if value is None and nullable:
        return
    if not isinstance(value, str):
        raise ValueError(f"{name} must be an ISO-8601 string")
    try:
        datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError(f"{name} must be ISO-8601") from exc


def parse_course(payload: Mapping[str, Any]) -> Course:
    if not isinstance(payload, Mapping) or payload.get("schema_version") != COURSE_SCHEMA:
        raise ValueError("unsupported Course schema_version")
    course_id, frequency, corners = payload.get("course_id"), payload.get("pinger_freq_hz"), payload.get("corners")
    if not isinstance(course_id, str) or not course_id:
        raise ValueError("Course course_id must be non-empty")
    if not isinstance(frequency, int) or isinstance(frequency, bool) or not 1 <= frequency <= 1_000_000:
        raise ValueError("Course pinger_freq_hz must be an integer in range")
    if not isinstance(corners, list) or len(corners) < 4:
        raise ValueError("Course requires a closed polygon with at least three vertices")
    normalized = []
    for item in corners:
        if not isinstance(item, Mapping):
            raise ValueError("Course corner must be an object")
        latitude, longitude = item.get("latitude"), item.get("longitude")
        if not isinstance(latitude, (int, float)) or not isinstance(longitude, (int, float)) or not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError("Course corner latitude/longitude is invalid")
        normalized.append((float(latitude), float(longitude)))
    if normalized[0] != normalized[-1]:
        raise ValueError("Course polygon must be closed")
    _timestamp(payload.get("timestamp"), "Course timestamp")
    _timestamp(payload.get("source_sent_at"), "Course source_sent_at", nullable=True)
    canonical = json.dumps(dict(payload), sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return Course(course_id, frequency, tuple(normalized), hashlib.sha256(canonical.encode()).hexdigest())


def parse_run_start(payload: Mapping[str, Any]) -> tuple[int, str]:
    if not isinstance(payload, Mapping) or payload.get("command") != "begin":
        raise ValueError("RunStart command must be begin")
    if payload.get("schema_version") not in RUN_SCHEMAS:
        raise ValueError("unsupported RunStart schema_version")
    run_id, command_id = payload.get("run_id"), payload.get("command_id")
    if not isinstance(run_id, int) or isinstance(run_id, bool) or run_id <= 0:
        raise ValueError("RunStart run_id must be positive")
    if not isinstance(command_id, str) or not command_id:
        raise ValueError("RunStart command_id must be non-empty")
    return run_id, command_id


def mission_start_permitted(*, run_start_received: bool, course_fsm_accepted: bool,
                            local_safety_ready: bool) -> bool:
    """Pure start gate used by the formal node.

    A Course being forwarded to the UAV is deliberately not an input here:
    the formal FSM can start MAP only after it has locally accepted a Course,
    received a valid RunStart, and received a real local-safety-ready input.
    """
    return run_start_received and course_fsm_accepted and local_safety_ready


def course_status(*, course: Course | None, fsm_accepted: bool,
                  uav_applied: bool) -> dict[str, Any]:
    """Build the unambiguous Course portion of status-v2.

    ``applied`` is retained for the pre-existing FSM-local meaning.  Consumers
    that need aircraft confirmation must use ``uav_applied`` instead.
    """
    return {
        "loaded": course is not None,
        "applied": fsm_accepted,
        "fsm_accepted": fsm_accepted,
        "uav_applied": uav_applied,
        "uav_application_state": "CONFIRMED" if uav_applied else "NOT_CONFIRMED",
        "course_id": course.course_id if course else None,
        "fingerprint": course.fingerprint if course else None,
    }


def classify_error(error: Any, *, task: str | None, safety_state: str) -> tuple[str, str]:
    """Return severity/action; semantic code alone never forces a decision."""
    if error in (None, "", "NONE"):
        return "INFO", "NONE"
    text = str(error).upper()
    if safety_state in {"SAFE_STOP", "E_STOP"} or any(word in text for word in ("E_STOP", "COLLISION", "CRITICAL")):
        return "STOP", "SAFE_STOP"
    if any(word in text for word in ("COMMUNICATION_LOST", "STALE_DATA", "TIMEOUT")):
        return "HOLD", "SAFE_HOLD"
    if task in {"MAP", "TASK1", "TASK3", "TASK4"}:
        return "WARN", "REPORT"
    return "INFO", "REPORT"
