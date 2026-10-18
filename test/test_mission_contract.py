import unittest

from mission_contract import (classify_error, course_status, mission_start_permitted,
                              parse_course, parse_run_start)


def course():
    return {
        "schema_version": "twave-course-config-v1", "course_id": "RX-A",
        "pinger_freq_hz": 27000,
        "corners": [{"latitude": 1, "longitude": 2}, {"latitude": 1, "longitude": 3},
                    {"latitude": 2, "longitude": 3}, {"latitude": 1, "longitude": 2}],
        "source_sent_at": None, "timestamp": "2026-10-18T00:00:00Z",
    }


class MissionContractTests(unittest.TestCase):
    def test_course_contract_rejects_open_polygon_and_has_stable_identity(self):
        self.assertEqual(parse_course(course()).fingerprint, parse_course(course()).fingerprint)
        invalid = course(); invalid["corners"] = invalid["corners"][:-1]
        with self.assertRaisesRegex(ValueError, "closed"):
            parse_course(invalid)

    def test_runstart_and_error_classification_are_explicit(self):
        self.assertEqual(parse_run_start({"schema_version": "twave-mission-dryrun-v1", "run_id": 42, "command_id": "r1", "command": "begin"}), (42, "r1"))
        self.assertEqual(classify_error("COMMUNICATION_LOST", task="TASK1", safety_state="READY"), ("HOLD", "SAFE_HOLD"))
        self.assertEqual(classify_error("vision degraded", task="TASK3", safety_state="READY"), ("WARN", "REPORT"))
        self.assertEqual(classify_error("collision", task="TASK4", safety_state="READY"), ("STOP", "SAFE_STOP"))

    def test_course_and_runstart_orders_remain_gated_by_real_local_safety(self):
        # Course-first and RunStart-first are equivalent: neither can start MAP
        # alone, and simulated/real readiness must be supplied explicitly.
        self.assertFalse(mission_start_permitted(run_start_received=False, course_fsm_accepted=True, local_safety_ready=True))
        self.assertFalse(mission_start_permitted(run_start_received=True, course_fsm_accepted=False, local_safety_ready=True))
        self.assertFalse(mission_start_permitted(run_start_received=True, course_fsm_accepted=True, local_safety_ready=False))
        self.assertTrue(mission_start_permitted(run_start_received=True, course_fsm_accepted=True, local_safety_ready=True))

    def test_course_status_exposes_local_acceptance_without_claiming_uav_ack(self):
        status = course_status(course=parse_course(course()), fsm_accepted=True, uav_applied=False)
        self.assertEqual(status["course_id"], "RX-A")
        self.assertTrue(status["fsm_accepted"])
        self.assertFalse(status["uav_applied"])
        self.assertEqual(status["uav_application_state"], "NOT_CONFIRMED")
