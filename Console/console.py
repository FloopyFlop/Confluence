#!/usr/bin/env python3
"""
Confluence Remote Console

Connects to a running Confluence orchestrator over TCP to stream logs and send
commands (fault injection, parameter updates, and service listing).

Dependencies:
- Python 3.8+
- No ROS2 required

Usage examples:
  python3 Console/console.py --host 192.168.1.50 --port 9000

Commands (type at the prompt):
  list
  fault <index>              (1-4)
  clear                       (clears forced fault)
  inject PARAM=VALUE ...      (publishes params to inject service)
  send {json}                 (send raw JSON command)
  quit
"""

import argparse
import json
import socket
import sys
import threading


def _print(line):
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


def _parse_command(line):
    line = line.strip()
    if not line:
        return None
    if line.startswith("send "):
        raw = line[len("send "):].strip()
        try:
            return json.loads(raw)
        except Exception:
            return {"cmd": "invalid", "raw": raw}
    parts = line.split()
    cmd = parts[0].lower()
    if cmd == "list":
        return {"cmd": "list"}
    if cmd in ("fault", "inject_fault"):
        if len(parts) > 1:
            try:
                return {"cmd": "fault", "fault_index": int(parts[1])}
            except Exception:
                return {"cmd": "fault", "fault_label": " ".join(parts[1:])}
        return {"cmd": "fault"}
    if cmd in ("clear", "clear_fault"):
        return {"cmd": "clear_fault"}
    if cmd in ("inject", "set_params"):
        params = {}
        for item in parts[1:]:
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            try:
                if "." in value:
                    cast_val = float(value)
                else:
                    cast_val = int(value)
            except Exception:
                cast_val = value
            params[key] = cast_val
        return {"cmd": "set_params", "params": params}
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
                _print(line)
                continue

            msg_type = payload.get("type")
            if msg_type == "log":
                service = payload.get("service", "?")
                text = payload.get("line", "")
                _print(f"[{service}] {text}")
            elif msg_type == "services":
                services = ", ".join(payload.get("services", []))
                _print(f"Services: {services}")
            elif msg_type in ("info", "ack", "error", "event"):
                _print(f"{msg_type.upper()}: {payload}")
            else:
                _print(str(payload))


def main():
    parser = argparse.ArgumentParser(description="Confluence Remote Console")
    parser.add_argument("--host", required=True, help="Drone/orchestrator IP")
    parser.add_argument("--port", type=int, required=True, help="Orchestrator console port")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))

    reader_thread = threading.Thread(target=_reader, args=(sock,), daemon=True)
    reader_thread.start()

    _print("Connected. Type 'list', 'fault 1', 'clear', 'inject PARAM=VALUE', or 'quit'.")
    while True:
        try:
            line = input("console> ")
        except EOFError:
            break
        cmd = _parse_command(line)
        if not cmd:
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
