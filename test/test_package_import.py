import unittest

from mission_contract import COURSE_SCHEMA, STATUS_SCHEMA


class PackageImportTests(unittest.TestCase):
    def test_contract_modules_are_importable(self):
        self.assertEqual(COURSE_SCHEMA, "twave-course-config-v1")
        self.assertEqual(STATUS_SCHEMA, "twave-mission-status-v2")
