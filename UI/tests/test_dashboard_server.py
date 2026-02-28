import json
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dashboard_server import DashboardState  # noqa: E402


def _topic_frame(topic: str, payload: dict):
    return {
        "type": "topic",
        "topic": topic,
        "message": {"data": json.dumps(payload)},
    }


class DashboardStateTests(unittest.TestCase):
    def test_fault_to_fix_to_operational_transition(self):
        state = DashboardState("127.0.0.1", 9000, fix_duration_sec=0.1)

        state.ingest_frame(
            _topic_frame(
                "fault_detector/output",
                {
                    "fault": True,
                    "fault_label": "motor 2 malfunction",
                    "source": "model",
                    "confidence": 0.77,
                    "raw_pred": 2,
                    "stable_pred": 2,
                    "parameters": {"PWM_MAIN_FUNC1": 102},
                },
            )
        )
        snap = state.snapshot()
        self.assertEqual(snap["derived"]["current_status"], "fault_detected")
        self.assertEqual(snap["fault"]["active"]["label"], "motor 2 malfunction")

        state.ingest_frame(
            _topic_frame(
                "px4_injector/status",
                {
                    "source": "fault_detector_model_landing_fix",
                    "results": {"PWM_MAIN_FUNC1": True},
                    "details": {
                        "PWM_MAIN_FUNC1": {
                            "ok": True,
                            "actual": 102,
                        }
                    },
                },
            )
        )
        snap = state.snapshot()
        self.assertEqual(snap["derived"]["current_status"], "fault_fixing")
        self.assertEqual(snap["fault"]["last_fix"]["changes"][0]["param"], "PWM_MAIN_FUNC1")

        time.sleep(0.12)
        snap = state.snapshot()
        self.assertEqual(snap["derived"]["current_status"], "operational")

    def test_raw_mav_updates_telemetry_fields(self):
        state = DashboardState("127.0.0.1", 9000, fix_duration_sec=5.0)

        state.ingest_frame(
            _topic_frame(
                "mav/all_messages",
                {
                    "_msg_type": "HEARTBEAT",
                    "base_mode": 128,
                    "system_status": 4,
                    "_timestamp": 100.0,
                },
            )
        )
        state.ingest_frame(
            _topic_frame(
                "mav/all_messages",
                {
                    "_msg_type": "RC_CHANNELS",
                    "chan3_raw": 1420,
                    "_timestamp": 101.0,
                },
            )
        )
        state.ingest_frame(
            _topic_frame(
                "mav/all_messages",
                {
                    "_msg_type": "GLOBAL_POSITION_INT",
                    "lat": 421234567,
                    "lon": -761234567,
                    "relative_alt": 12450,
                    "_timestamp": 102.0,
                },
            )
        )

        snap = state.snapshot()
        self.assertTrue(snap["telemetry"]["armed"])
        self.assertEqual(snap["telemetry"]["throttle"], 1420.0)
        self.assertAlmostEqual(snap["telemetry"]["gps"]["lat"], 42.1234567)
        self.assertAlmostEqual(snap["telemetry"]["gps"]["lon"], -76.1234567)
        self.assertAlmostEqual(snap["telemetry"]["gps"]["alt_m"], 12.45)

    def test_connection_state_changes_emit_health(self):
        state = DashboardState("127.0.0.1", 9000, fix_duration_sec=5.0)
        state.on_connection_change(True)
        health = state.health()
        self.assertTrue(health["connected"])

        state.on_connection_change(False, "socket closed")
        health = state.health()
        self.assertFalse(health["connected"])
        self.assertEqual(health["last_error"], "socket closed")


if __name__ == "__main__":
    unittest.main()
