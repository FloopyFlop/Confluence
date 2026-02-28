#!/usr/bin/env python3
"""
Confluence demo dashboard server.

This server bridges the orchestrator TCP console stream to a browser UI.
It is read-only and intended for demo/observability use.

Usage:
  uv run UI/dashboard_server.py --orchestrator-host 127.0.0.1 --orchestrator-port 9000 --ui-port 8765
"""

from __future__ import annotations

import argparse
import copy
import json
import socket
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

WATCH_TOPICS = [
    ("fault_detector/output", "std_msgs/msg/String"),
    ("fault_detector/status", "std_msgs/msg/String"),
    ("fault_detector/diagnostics", "std_msgs/msg/String"),
    ("px4_injector/status", "std_msgs/msg/String"),
    ("mav/all_messages", "std_msgs/msg/String"),
]


def _now() -> float:
    return time.time()


def _safe_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _safe_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


class DashboardState:
    def __init__(self, orchestrator_host: str, orchestrator_port: int, fix_duration_sec: float) -> None:
        self._lock = threading.RLock()
        self._history_limit = 120
        self._raw_rate_window: deque[float] = deque(maxlen=512)
        self._confidence_history: deque[dict[str, float]] = deque(maxlen=40)
        self._throttle_history: deque[dict[str, float]] = deque(maxlen=40)
        now = _now()
        self._state: dict[str, Any] = {
            "meta": {
                "name": "Confluence Demo Dashboard",
                "version": "1.0.0",
                "fix_duration_sec": float(fix_duration_sec),
                "started_at": now,
                "frames_seen": 0,
            },
            "connection": {
                "orchestrator_host": orchestrator_host,
                "orchestrator_port": int(orchestrator_port),
                "connected": False,
                "last_connected_ts": None,
                "last_change_ts": now,
                "last_frame_ts": None,
                "last_error": None,
                "reconnect_count": 0,
            },
            "derived": {
                "current_status": "operational",
                "previous_status": None,
                "status_entered_at": now,
                "fixing_started_at": None,
                "fixing_ends_at": None,
            },
            "fault": {
                "active": None,
                "last_fix": None,
                "latest_output": None,
                "latest_status": None,
                "latest_diagnostics": None,
                "latest_injector_status": None,
            },
            "model": {
                "running": False,
                "loaded": None,
                "last_confidence": None,
                "last_inference_ts": None,
                "confidence_history": [],
            },
            "telemetry": {
                "armed": None,
                "gate_open": None,
                "throttle": None,
                "throttle_up": None,
                "system_status": None,
                "gps": {
                    "lat": None,
                    "lon": None,
                    "alt_m": None,
                },
                "local_position_ned": {
                    "x": None,
                    "y": None,
                    "z": None,
                },
                "groundspeed": None,
                "airspeed": None,
                "heading": None,
                "last_message_type": None,
                "last_message_ts": None,
                "message_counts": {},
                "raw_stream_rate_hz": 0.0,
                "inferences_run": 0,
                "batches_received": 0,
                "param_values": {},
                "throttle_history": [],
            },
            "log": [],
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._tick_locked()
            self._state["telemetry"]["raw_stream_rate_hz"] = self._compute_raw_rate_locked()
            self._state["telemetry"]["throttle_history"] = list(self._throttle_history)
            self._state["model"]["confidence_history"] = list(self._confidence_history)
            return copy.deepcopy(self._state)

    def health(self) -> dict[str, Any]:
        with self._lock:
            conn = self._state["connection"]
            return {
                "ok": True,
                "connected": bool(conn.get("connected")),
                "last_frame_ts": conn.get("last_frame_ts"),
                "last_error": conn.get("last_error"),
                "status": self._state["derived"]["current_status"],
                "raw_stream_rate_hz": self._compute_raw_rate_locked(),
            }

    def on_connection_change(self, connected: bool, error: str | None = None) -> None:
        with self._lock:
            now = _now()
            conn = self._state["connection"]
            was_connected = bool(conn.get("connected"))
            now_connected = bool(connected)
            conn["connected"] = now_connected
            conn["last_change_ts"] = now
            if now_connected:
                conn["last_connected_ts"] = now
                conn["last_error"] = None
                if not was_connected:
                    self._append_log_locked("connection", "Bridge connected", "Live topic stream active")
            else:
                if error:
                    conn["last_error"] = str(error)
                conn["reconnect_count"] = int(conn.get("reconnect_count", 0)) + 1
                if was_connected:
                    detail = str(error) if error else "connection dropped"
                    self._append_log_locked("connection", "Bridge disconnected", detail)

    def ingest_frame(self, frame: dict[str, Any]) -> None:
        with self._lock:
            conn = self._state["connection"]
            conn["last_frame_ts"] = _now()
            self._state["meta"]["frames_seen"] = int(self._state["meta"]["frames_seen"]) + 1

            frame_type = str(frame.get("type") or "")
            if frame_type == "topic":
                topic = str(frame.get("topic") or "")
                payload = self._extract_topic_payload(frame)
                if topic and payload is not None:
                    self._apply_topic_locked(topic, payload)
                return

            if frame_type in {"event", "error", "ack", "info"}:
                payload_text = json.dumps(frame, separators=(",", ":"))
                self._append_log_locked("bridge", frame_type.upper(), payload_text[:220])

    def _extract_topic_payload(self, frame: dict[str, Any]) -> dict[str, Any] | None:
        message = frame.get("message")
        candidate: Any = message
        if isinstance(message, dict) and "data" in message:
            candidate = message.get("data")

        if isinstance(candidate, dict):
            return candidate
        if isinstance(candidate, str):
            try:
                decoded = json.loads(candidate)
                if isinstance(decoded, dict):
                    return decoded
                return {"value": decoded}
            except Exception:
                return {"raw_text": candidate}
        return None

    def _apply_topic_locked(self, topic: str, payload: dict[str, Any]) -> None:
        if topic == "fault_detector/output":
            self._on_fault_output_locked(payload)
            return
        if topic == "fault_detector/status":
            self._on_fault_status_locked(payload)
            return
        if topic == "fault_detector/diagnostics":
            self._on_diagnostics_locked(payload)
            return
        if topic == "px4_injector/status":
            self._on_injector_status_locked(payload)
            return
        if topic == "mav/all_messages":
            self._on_raw_mav_locked(payload)
            return

    def _on_fault_output_locked(self, payload: dict[str, Any]) -> None:
        self._state["fault"]["latest_output"] = copy.deepcopy(payload)
        fault = bool(payload.get("fault"))
        label = str(payload.get("fault_label") or "nominal")
        confidence = _safe_float(payload.get("confidence"))

        if confidence is not None:
            now = _now()
            self._state["model"]["running"] = True
            self._state["model"]["last_confidence"] = confidence
            self._state["model"]["last_inference_ts"] = payload.get("timestamp", now)
            self._confidence_history.append({"t": now, "v": confidence})

        if fault:
            active = {
                "label": label,
                "source": str(payload.get("source") or "unknown"),
                "confidence": confidence,
                "raw_pred": payload.get("raw_pred"),
                "stable_pred": payload.get("stable_pred"),
                "parameters": payload.get("parameters") if isinstance(payload.get("parameters"), dict) else {},
                "timestamp": payload.get("timestamp", _now()),
                "raw_json": copy.deepcopy(payload),
            }
            self._state["fault"]["active"] = active
            self._set_status_locked("fault_detected", f"{label} ({active['source']})")
        else:
            self._state["fault"]["active"] = None
            if self._state["derived"]["current_status"] != "fault_fixing":
                self._set_status_locked("operational", "Fault output nominal")

    def _on_fault_status_locked(self, payload: dict[str, Any]) -> None:
        self._state["fault"]["latest_status"] = copy.deepcopy(payload)
        telemetry = self._state["telemetry"]
        model = self._state["model"]

        if "armed" in payload:
            telemetry["armed"] = bool(payload.get("armed"))
        if "gate_open" in payload:
            telemetry["gate_open"] = bool(payload.get("gate_open"))
        if "throttle_up" in payload:
            telemetry["throttle_up"] = bool(payload.get("throttle_up"))

        throttle = _safe_float(payload.get("throttle"))
        if throttle is not None:
            telemetry["throttle"] = throttle
            self._throttle_history.append({"t": _now(), "v": throttle})

        confidence = _safe_float(payload.get("confidence"))
        if confidence is not None:
            now = _now()
            model["running"] = True
            model["last_confidence"] = confidence
            model["last_inference_ts"] = payload.get("timestamp", now)
            self._confidence_history.append({"t": now, "v": confidence})

    def _on_diagnostics_locked(self, payload: dict[str, Any]) -> None:
        self._state["fault"]["latest_diagnostics"] = copy.deepcopy(payload)
        telemetry = self._state["telemetry"]
        model = self._state["model"]
        event = str(payload.get("event") or "")

        if "model_loaded" in payload:
            model["loaded"] = bool(payload.get("model_loaded"))
        if "batches_received" in payload:
            telemetry["batches_received"] = int(payload.get("batches_received") or 0)
        if "inferences_run" in payload:
            telemetry["inferences_run"] = int(payload.get("inferences_run") or 0)
        if "flight_gate_open" in payload:
            telemetry["gate_open"] = bool(payload.get("flight_gate_open"))
        if "armed" in payload:
            telemetry["armed"] = bool(payload.get("armed"))

        throttle = _safe_float(payload.get("throttle"))
        if throttle is not None:
            telemetry["throttle"] = throttle
            telemetry["throttle_up"] = bool(payload.get("throttle_up", throttle > 1100.0))
            self._throttle_history.append({"t": _now(), "v": throttle})

        if event == "inference":
            model["running"] = True
            model["last_inference_ts"] = payload.get("timestamp", _now())
            conf = _safe_float(payload.get("confidence"))
            if conf is not None:
                model["last_confidence"] = conf
                self._confidence_history.append({"t": _now(), "v": conf})

        notable = {
            "flight_gate_state": "Flight gate update",
            "model_fault_latched": "Model fault latched",
            "model_fix_applied_after_landing": "Model fix applied",
            "clear_restore_published": "Restore command published",
            "error": "Detector error",
            "probe_result": "Probe result",
        }
        if event in notable:
            self._append_log_locked("diagnostic", notable[event], json.dumps(payload, separators=(",", ":"))[:220])

    def _on_injector_status_locked(self, payload: dict[str, Any]) -> None:
        self._state["fault"]["latest_injector_status"] = copy.deepcopy(payload)
        telemetry = self._state["telemetry"]
        active_fault = self._state["fault"].get("active")
        active_params = active_fault.get("parameters", {}) if isinstance(active_fault, dict) else {}

        details = payload.get("details") if isinstance(payload.get("details"), dict) else {}
        results = payload.get("results") if isinstance(payload.get("results"), dict) else {}
        changes: list[dict[str, Any]] = []

        if details:
            for param in sorted(details.keys()):
                detail_raw = details.get(param)
                detail = detail_raw if isinstance(detail_raw, dict) else {}
                previous_value = telemetry["param_values"].get(param)
                actual = detail.get("actual")
                target_value = detail.get("target")
                if target_value is None and isinstance(active_params, dict):
                    target_value = active_params.get(param)
                new_value = actual if actual is not None else target_value
                if actual is not None:
                    telemetry["param_values"][param] = actual
                ok_value = detail.get("ok")
                if ok_value is None:
                    ok_value = results.get(param)
                changes.append(
                    {
                        "param": param,
                        "previous_value": previous_value,
                        "new_value": new_value,
                        "target_value": target_value,
                        "ok": bool(ok_value) if ok_value is not None else None,
                        "reason": detail.get("reason"),
                    }
                )
        elif results:
            for param in sorted(results.keys()):
                changes.append(
                    {
                        "param": param,
                        "previous_value": telemetry["param_values"].get(param),
                        "new_value": None,
                        "target_value": None,
                        "ok": bool(results.get(param)),
                        "reason": None,
                    }
                )

        last_fix = {
            "timestamp": _now(),
            "source": str(payload.get("source") or "injector"),
            "queued": bool(payload.get("queued", False)),
            "armed": payload.get("armed"),
            "changes": changes,
            "raw_json": copy.deepcopy(payload),
        }
        self._state["fault"]["last_fix"] = last_fix

        source = str(last_fix.get("source") or "")
        if source.startswith("fault_detector"):
            now = _now()
            self._set_status_locked("fault_fixing", f"{source} ({len(changes)} params)")
            self._state["derived"]["fixing_started_at"] = now
            self._state["derived"]["fixing_ends_at"] = now + float(self._state["meta"]["fix_duration_sec"])

    def _on_raw_mav_locked(self, payload: dict[str, Any]) -> None:
        telemetry = self._state["telemetry"]
        now = _now()
        self._raw_rate_window.append(now)

        msg_type = str(payload.get("_msg_type") or payload.get("msg_type") or "").upper()
        if msg_type:
            counts = telemetry["message_counts"]
            counts[msg_type] = int(counts.get(msg_type, 0)) + 1
            telemetry["last_message_type"] = msg_type
        telemetry["last_message_ts"] = payload.get("_timestamp", now)

        if msg_type == "HEARTBEAT":
            base_mode = _safe_int(payload.get("base_mode"))
            if base_mode is not None:
                telemetry["armed"] = bool(base_mode & 0x80)
            telemetry["system_status"] = payload.get("system_status")

        elif msg_type == "RC_CHANNELS":
            throttle = self._extract_throttle(payload)
            if throttle is not None:
                telemetry["throttle"] = throttle
                telemetry["throttle_up"] = throttle > 1100.0
                self._throttle_history.append({"t": now, "v": throttle})

        elif msg_type == "GLOBAL_POSITION_INT":
            lat = _safe_float(payload.get("lat"))
            lon = _safe_float(payload.get("lon"))
            rel_alt = _safe_float(payload.get("relative_alt"))
            alt = _safe_float(payload.get("alt"))
            if lat is not None:
                telemetry["gps"]["lat"] = lat / 1e7
            if lon is not None:
                telemetry["gps"]["lon"] = lon / 1e7
            if rel_alt is not None:
                telemetry["gps"]["alt_m"] = rel_alt / 1000.0
            elif alt is not None:
                telemetry["gps"]["alt_m"] = alt / 1000.0

        elif msg_type == "LOCAL_POSITION_NED":
            telemetry["local_position_ned"]["x"] = _safe_float(payload.get("x"))
            telemetry["local_position_ned"]["y"] = _safe_float(payload.get("y"))
            telemetry["local_position_ned"]["z"] = _safe_float(payload.get("z"))

        elif msg_type == "VFR_HUD":
            telemetry["groundspeed"] = _safe_float(payload.get("groundspeed"))
            telemetry["airspeed"] = _safe_float(payload.get("airspeed"))
            telemetry["heading"] = _safe_float(payload.get("heading"))

    def _extract_throttle(self, payload: dict[str, Any]) -> float | None:
        direct = _safe_float(payload.get("chan3_raw"))
        if direct is not None:
            return direct
        channels = payload.get("channels")
        if isinstance(channels, (list, tuple)) and len(channels) >= 3:
            return _safe_float(channels[2])
        return None

    def _set_status_locked(self, new_status: str, detail: str | None = None) -> None:
        derived = self._state["derived"]
        current = derived.get("current_status")
        if current == new_status:
            return

        now = _now()
        derived["previous_status"] = current
        derived["current_status"] = new_status
        derived["status_entered_at"] = now
        if new_status != "fault_fixing":
            derived["fixing_started_at"] = None
            derived["fixing_ends_at"] = None

        self._append_log_locked("status", f"{current} -> {new_status}", detail or "")

    def _tick_locked(self) -> None:
        derived = self._state["derived"]
        if derived.get("current_status") != "fault_fixing":
            return

        ends_at = _safe_float(derived.get("fixing_ends_at"))
        if ends_at is None:
            return
        if _now() < ends_at:
            return

        self._state["fault"]["active"] = None
        self._set_status_locked("operational", "Fix animation complete")

    def _compute_raw_rate_locked(self) -> float:
        now = _now()
        while self._raw_rate_window and (now - self._raw_rate_window[0]) > 5.0:
            self._raw_rate_window.popleft()

        if len(self._raw_rate_window) < 2:
            return 0.0

        window = self._raw_rate_window[-1] - self._raw_rate_window[0]
        if window <= 0.0:
            return float(len(self._raw_rate_window))
        return round(len(self._raw_rate_window) / window, 2)

    def _append_log_locked(self, kind: str, title: str, detail: str) -> None:
        entry = {
            "timestamp": _now(),
            "kind": kind,
            "title": title,
            "detail": detail,
        }
        logs = self._state["log"]
        logs.insert(0, entry)
        if len(logs) > self._history_limit:
            del logs[self._history_limit :]


class OrchestratorBridge:
    def __init__(self, state: DashboardState, host: str, port: int) -> None:
        self._state = state
        self._host = host
        self._port = int(port)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket_lock = threading.Lock()
        self._socket: socket.socket | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="orchestrator-bridge", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._close_socket()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _install_socket(self, sock: socket.socket | None) -> None:
        with self._socket_lock:
            self._socket = sock

    def _close_socket(self) -> None:
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass

    def _send_watch_commands(self, sock: socket.socket) -> None:
        for topic, type_name in WATCH_TOPICS:
            payload = {"cmd": "watch", "topic": topic, "type": type_name}
            line = json.dumps(payload) + "\n"
            sock.sendall(line.encode("utf-8"))

    def _run(self) -> None:
        backoff = 1.0
        while not self._stop_event.is_set():
            reader = None
            sock = None
            try:
                sock = socket.create_connection((self._host, self._port), timeout=5.0)
                self._install_socket(sock)
                self._state.on_connection_change(True)
                self._send_watch_commands(sock)
                reader = sock.makefile("r", encoding="utf-8", errors="replace")
                backoff = 1.0

                while not self._stop_event.is_set():
                    line = reader.readline()
                    if not line:
                        raise ConnectionError("orchestrator closed the connection")
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        frame = json.loads(line)
                    except Exception:
                        self._state.ingest_frame(
                            {
                                "type": "error",
                                "message": "non-json frame",
                                "raw": line[:220],
                            }
                        )
                        continue
                    if isinstance(frame, dict):
                        self._state.ingest_frame(frame)

            except Exception as exc:
                self._state.on_connection_change(False, str(exc))
                if self._stop_event.is_set():
                    break
                time.sleep(backoff)
                backoff = min(backoff * 1.8, 5.0)
            finally:
                if reader is not None:
                    try:
                        reader.close()
                    except Exception:
                        pass
                self._close_socket()


class DashboardHandler(SimpleHTTPRequestHandler):
    state_ref: DashboardState
    static_dir: Path

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(self.static_dir), **kwargs)

    def log_message(self, _fmt: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        route = parsed.path

        if route == "/api/state":
            self._send_json(self.state_ref.snapshot())
            return

        if route == "/health":
            self._send_json(self.state_ref.health())
            return

        if route in ("/", ""):
            self.path = "/index.html"

        super().do_GET()

    def _send_json(self, payload: dict[str, Any], status: int = HTTPStatus.OK) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def build_handler(state: DashboardState, static_dir: Path) -> type[DashboardHandler]:
    class _Handler(DashboardHandler):
        pass

    _Handler.state_ref = state
    _Handler.static_dir = static_dir

    return _Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Confluence demo dashboard")
    parser.add_argument("--orchestrator-host", default="127.0.0.1", help="Orchestrator TCP host")
    parser.add_argument("--orchestrator-port", type=int, default=9000, help="Orchestrator TCP port")
    parser.add_argument("--ui-host", default="127.0.0.1", help="Dashboard bind host")
    parser.add_argument("--ui-port", type=int, default=8765, help="Dashboard bind port")
    parser.add_argument(
        "--fix-duration-sec",
        type=float,
        default=5.0,
        help="Visual fault-fixing duration in seconds",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    static_dir = Path(__file__).resolve().parent / "static"
    if not (static_dir / "index.html").exists():
        raise FileNotFoundError(f"Missing UI assets under {static_dir}")

    state = DashboardState(
        orchestrator_host=str(args.orchestrator_host),
        orchestrator_port=int(args.orchestrator_port),
        fix_duration_sec=float(args.fix_duration_sec),
    )
    bridge = OrchestratorBridge(state, host=str(args.orchestrator_host), port=int(args.orchestrator_port))
    bridge.start()

    handler_cls = build_handler(state, static_dir)
    server = ThreadingHTTPServer((str(args.ui_host), int(args.ui_port)), handler_cls)

    print(f"[dashboard] listening on http://{args.ui_host}:{args.ui_port}")
    print(
        "[dashboard] streaming from "
        f"{args.orchestrator_host}:{args.orchestrator_port} "
        f"(fix animation {args.fix_duration_sec:.1f}s)"
    )

    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
