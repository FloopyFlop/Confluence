#!/usr/bin/env python3
"""
Induce Fault Hook

Send a one-off fault injection command to a running orchestrator console.

Dependencies:
- Python 3.8+

Usage:
  python3 confluence/hooks/induce_fault.py --host <DRONE_IP> --port 9000 --motor 1
  python3 confluence/hooks/induce_fault.py --host 127.0.0.1 --port 9000 --motor 2

Notes:
- Requires the orchestrator to be running with --console-port enabled.
- This hook sends a command to the fault_detector via orchestrator.
"""

import argparse
import json
import socket
import sys


def _send(host, port, payload):
    data = json.dumps(payload) + "\n"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    sock.sendall(data.encode("utf-8"))
    sock.close()


def main():
    parser = argparse.ArgumentParser(description="Induce fault via orchestrator console")
    parser.add_argument("--host", required=True, help="Orchestrator host (drone IP)")
    parser.add_argument("--port", type=int, required=True, help="Orchestrator console port")
    parser.add_argument("--motor", type=int, required=True, help="Motor index to induce fault (1-4)")
    args = parser.parse_args()

    if args.motor < 1 or args.motor > 4:
        raise SystemExit("motor must be between 1 and 4")

    payload = {
        "cmd": "fault",
        "fault_index": args.motor,
    }
    _send(args.host, args.port, payload)
    sys.stdout.write("Sent fault command\n")


if __name__ == "__main__":
    main()
