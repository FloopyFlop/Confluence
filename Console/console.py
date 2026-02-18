#!/usr/bin/env python3
"""
Confluence Remote Console

Connects to a running Confluence orchestrator over TCP to stream logs and send
commands (fault injection, parameter updates, topic watch/publish, and probes).

Dependencies:
- Python 3.8+
- No ROS2 required

Usage examples:
  python3 Console/console.py --host 192.168.1.50 --port 9000

Commands (type at the prompt):
  list
  probe                      (request fault-detector probe diagnostics)
  fault <index>              (1-4)
  clear                      (clears forced fault; restore params using detector defaults)
  clear no-restore           (clear without restore)
  clear now                  (clear and restore immediately, even if armed)
  clear now no-restore       (force-clear only)
  inject PARAM=VALUE ...
  motortest <on|off>         (sets COM_MOT_TEST_EN=1/0)
  watch <topic> <type>
  unwatch <topic>
  pub <topic> <type> <json>
  send {json}
  quit
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading


PROMPT = "console> "
PRINT_LOCK = threading.Lock()


def _print(line: str):
    with PRINT_LOCK:
        sys.stdout.write(line + "\n")
        sys.stdout.flush()


def _print_async(line: str):
    # Keep interactive input stable while background lines arrive.
    with PRINT_LOCK:
        sys.stdout.write("\r\033[2K")
        sys.stdout.write(line + "\n")
        # Re-render prompt without duplicating partially typed commands.
        sys.stdout.write(PROMPT)
        sys.stdout.flush()


def _print_help():
    _print("Commands:")
    _print("  list")
    _print("  probe")
    _print("  fault <index>")
    _print("  clear")
    _print("  clear no-restore")
    _print("  clear now")
    _print("  clear now no-restore")
    _print("  inject PARAM=VALUE ...")
    _print("  motortest <on|off>")
    _print("  watch <topic> <type>")
    _print("  unwatch <topic>")
    _print("  pub <topic> <type> <json>")
    _print("  send {json}")
    _print("  quit")


def _parse_command(line: str):
    line = line.strip()
    if not line:
        return None
    if line == "help":
        return {"cmd": "help"}
    if line.startswith("send "):
        raw = line[len("send ") :].strip()
        try:
            return json.loads(raw)
        except Exception:
            return {"cmd": "invalid", "raw": raw}

    parts = line.split()
    cmd = parts[0].lower()
    if cmd == "list":
        return {"cmd": "list"}
    if cmd in ("probe", "run_probe"):
        return {"cmd": "probe"}
    if cmd in ("fault", "inject_fault"):
        if len(parts) > 1:
            try:
                return {"cmd": "fault", "fault_index": int(parts[1])}
            except Exception:
                return {"cmd": "fault", "fault_label": " ".join(parts[1:])}
        return {"cmd": "fault"}
    if cmd in ("clear", "clear_fault"):
        out = {"cmd": "clear_fault"}
        for token in parts[1:]:
            t = token.strip().lower()
            if t in ("now", "force", "immediate"):
                out["defer_until_disarmed"] = False
            elif t in ("defer", "safe"):
                out["defer_until_disarmed"] = True
            elif t in ("no-restore", "norestore", "keep"):
                out["restore"] = False
            elif t in ("restore",):
                out["restore"] = True
        return out
    if cmd in ("inject", "set_params"):
        params = {}
        for item in parts[1:]:
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            try:
                cast_val = float(value) if "." in value else int(value)
            except Exception:
                cast_val = value
            params[key] = cast_val
        return {"cmd": "set_params", "params": params}
    if cmd == "motortest":
        if len(parts) < 2:
            return {"cmd": "motortest"}
        state = parts[1].strip().lower()
        if state in ("on", "enable", "enabled", "1", "true"):
            return {"cmd": "set_params", "params": {"COM_MOT_TEST_EN": 1}}
        if state in ("off", "disable", "disabled", "0", "false"):
            return {"cmd": "set_params", "params": {"COM_MOT_TEST_EN": 0}}
        return {"cmd": "motortest", "value": state}
    if cmd == "watch":
        if len(parts) < 3:
            return {"cmd": "watch"}
        return {"cmd": "watch", "topic": parts[1], "type": parts[2]}
    if cmd == "unwatch":
        if len(parts) < 2:
            return {"cmd": "unwatch"}
        return {"cmd": "unwatch", "topic": parts[1]}
    if cmd in ("pub", "publish"):
        split = line.split(maxsplit=3)
        if len(split) < 4:
            return {"cmd": "publish"}
        _, topic, type_str, raw_json = split
        try:
            data = json.loads(raw_json)
        except Exception:
            data = raw_json
        return {"cmd": "publish", "topic": topic, "type": type_str, "data": data}
    if cmd in ("quit", "exit"):
        return {"cmd": "quit"}
    return {"cmd": cmd}


def _reader(sock):
    with sock.makefile("r", encoding="utf-8", errors="replace") as reader:
        for line in reader:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except Exception:
                _print_async(line)
                continue

            msg_type = payload.get("type")
            if msg_type == "log":
                service = payload.get("service", "?")
                text = payload.get("line", "")
                _print_async(f"[{service}] {text}")
            elif msg_type == "services":
                services = ", ".join(payload.get("services", []))
                _print_async(f"Services: {services}")
            elif msg_type in ("info", "ack", "error", "event"):
                _print_async(f"{msg_type.upper()}: {payload}")
            elif msg_type == "topic":
                topic = payload.get("topic", "?")
                message = payload.get("message")
                _print_async(f"[{topic}] {message}")
            else:
                _print_async(str(payload))


def main():
    parser = argparse.ArgumentParser(description="Confluence Remote Console")
    parser.add_argument("--host", default="127.0.0.1", help="Drone/orchestrator IP")
    parser.add_argument("--port", type=int, default=9000, help="Orchestrator console port")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))

    reader_thread = threading.Thread(target=_reader, args=(sock,), daemon=True)
    reader_thread.start()

    _print("Connected. Type 'help' for commands.")
    while True:
        try:
            line = input(PROMPT)
        except KeyboardInterrupt:
            _print("")
            break
        except EOFError:
            break
        cmd = _parse_command(line)
        if not cmd:
            continue
        if cmd.get("cmd") == "help":
            _print_help()
            continue
        if cmd.get("cmd") == "quit":
            break
        payload = json.dumps(cmd) + "\n"
        try:
            sock.sendall(payload.encode("utf-8"))
        except Exception as exc:
            _print(f"ERROR: {exc}")
            break

    try:
        sock.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
