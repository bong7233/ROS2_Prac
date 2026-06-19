import unittest

from amr_tools import fault_scenario, health_report


class TestImports(unittest.TestCase):
    def test_health_report_mode_names_include_manual_mode(self):
        self.assertEqual(
            health_report.MODE_NAMES[health_report.RobotState.MODE_MANUAL],
            "MANUAL",
        )

    def test_fault_scenario_client_exposes_field_helpers(self):
        self.assertTrue(hasattr(fault_scenario.FaultScenarioClient, "set_input"))
        self.assertTrue(hasattr(fault_scenario.FaultScenarioClient, "inject_motor_fault"))
        self.assertTrue(hasattr(fault_scenario.FaultScenarioClient, "reset_system_fault"))
