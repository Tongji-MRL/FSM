import tempfile
import unittest
from pathlib import Path

from task4_guard import Task4Guard


def assistance(*, run_id=42, command_id="a1"):
    return {"schema_version": "twave-mission-task4-v1", "run_id": run_id, "command_id": command_id, "vehicle": "uav", "task": "TASK4", "command": {"type": "ASSISTANCE_REQUEST", "command_seq": 9, "position": {"latitude": 31.0, "longitude": 121.0}}}


class Task4GuardTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.guard = Task4Guard(Path(self.dir.name) / "task4.json")

    def tearDown(self):
        self.dir.cleanup()

    def test_assistance_needs_both_pause_acks_and_matching_checkpoints(self):
        self.assertEqual(self.guard.accept(assistance(), active_run_id=42, planner_state="TASK1_EXECUTE"), "ACCEPTED_WAITING_SAFE_PAUSE")
        self.assertTrue(self.guard.pause_ack("uav", run_id=42, command_id="a1", accepted=True))
        self.assertTrue(self.guard.pause_ack("usv", run_id=42, command_id="a1", accepted=True))
        checkpoint = {"run_id": 42, "source": "uav", "vehicle": "uav", "pause_command_id": "a1", "timestamp": "2026-10-18T00:00:00Z", "task": "TASK1", "progress": "PAUSED"}
        self.assertTrue(self.guard.checkpoint("uav", checkpoint))
        self.assertFalse(self.guard.can_start())
        checkpoint.update({"source": "usv", "vehicle": "usv"})
        self.assertTrue(self.guard.checkpoint("usv", checkpoint))
        self.assertTrue(self.guard.can_start())
        self.guard.mark_started()
        self.assertEqual(self.guard.record["phase"], "EXECUTING")
        self.assertEqual(self.guard.execution_safety_status(), ("TASK4_EXECUTING", []))

    def test_start_gate_rejects_stale_or_mismatched_persisted_inputs(self):
        self.guard.accept(assistance(), active_run_id=42, planner_state="TASK1_EXECUTE")
        self.assertFalse(self.guard.pause_ack("uav", run_id=42, command_id="wrong", accepted=True))
        self.assertFalse(self.guard.pause_ack("uav", run_id=41, command_id="a1", accepted=True))
        self.assertTrue(self.guard.pause_ack("uav", run_id=42, command_id="a1", accepted=True))
        self.assertTrue(self.guard.pause_ack("usv", run_id=42, command_id="a1", accepted=True))
        self.assertTrue(self.guard.checkpoint("uav", {
            "run_id": 42, "source": "uav", "vehicle": "uav", "pause_command_id": "a1",
            "timestamp": "2026-10-18T00:00:00Z", "task": "MAP", "progress": "MAP_PAUSED",
        }))
        self.assertTrue(self.guard.checkpoint("usv", {
            "run_id": 41, "source": "usv", "vehicle": "usv", "pause_command_id": "a1",
            "timestamp": "2026-10-18T00:00:00Z", "task": "MAP", "progress": "MAP_PAUSED",
        }) is False)
        self.assertFalse(self.guard.can_start())
        # Persisted state must also be revalidated after restart/tampering.
        self.guard.record["checkpoints"]["usv"] = {
            "run_id": 42, "source": "usv", "vehicle": "usv", "pause_command_id": "wrong",
            "timestamp": "2026-10-18T00:00:00Z", "task": "MAP", "progress": "MAP_PAUSED",
        }
        self.assertFalse(self.guard.can_start())

    def test_duplicate_or_old_run_assistance_cannot_reset_the_gate(self):
        self.guard.accept(assistance(), active_run_id=42, planner_state="MAP_EXECUTE")
        self.assertEqual(
            self.guard.accept(assistance(), active_run_id=42, planner_state="MAP_EXECUTE"),
            "DUPLICATE",
        )
        self.assertEqual(self.guard.record["phase"], "WAITING_SAFE_PAUSE")
        with self.assertRaisesRegex(ValueError, "run_id"):
            self.guard.accept(
                assistance(run_id=41, command_id="old-run"),
                active_run_id=42,
                planner_state="MAP_EXECUTE",
            )

    def test_readiness_confirm_rejects_wrong_report_sequence(self):
        self.guard.record = {"phase": "READY_REPORTED", "run_id": 42, "vehicle": "uav", "command_seq": 9, "readiness_event_id": "a1:readiness"}
        notice = {"schema_version": "twave-mission-task4-v1", "run_id": 42, "command_id": "a1:readiness", "vehicle": "uav", "task": "TASK4", "command": {"type": "READINESS_REPORT_PUBLISHED", "command_seq": 9, "report_seq": 7, "readiness_event_id": "a1:readiness", "internal_origin": "ocs"}}
        self.assertEqual(self.guard.accept(notice, active_run_id=42, planner_state="TASK4_EXECUTE"), "READINESS_REPORT_RECORDED")
        payload = {"run_id": 42, "vehicle": "uav", "command": {"type": "READINESS_CONFIRM", "report_seq": 7, "original_command_seq": 9}}
        self.assertFalse(self.guard.confirm(payload, readiness_report_seq=8, safety_state="READY"))
        self.assertTrue(self.guard.confirm(payload, readiness_report_seq=7, safety_state="READY"))
        self.assertFalse(self.guard.confirm(payload, readiness_report_seq=7, safety_state="SAFE_HOLD"))
