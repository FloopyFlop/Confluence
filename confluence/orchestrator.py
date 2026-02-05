#!/usr/bin/env python3
"""
Confluence Orchestrator

A simple terminal-based launcher for the four Confluence services. It spawns
subprocesses, multiplexes their output, and lets you attach to any service for
interactive IO.

Usage examples:
  # Start all services
  ros2 run confluence orchestrator --all

  # Start specific services
  ros2 run confluence orchestrator --services firehose uniform_pump fault_detector

  # Pass args to a specific service
  ros2 run confluence orchestrator --all \
    --firehose-args "--sitl --stream-rate 200" \
    --uniform-pump-args "--ros-args -p condensation_mode:=multi_step -p multi_step_count:=3" \
    --fault-detector-args "--ros-args -p model_path:=/path/to/model.ckpt"

  # Write combined logs to a file
  ros2 run confluence orchestrator --all --log-file /tmp/confluence_run.txt

Interactive commands (type in the orchestrator prompt):
  list                    Show running services
  firehose                Attach to firehose output/IO
  uniform_pump            Attach to uniform_pump output/IO
  inject                  Attach to inject output/IO
  fault_detector          Attach to fault_detector output/IO
  attach <name>           Same as typing the service name
  help                    Show help
  quit                    Stop all services and exit

While attached:
  - All keystrokes go to the attached service.
  - Press Ctrl-] to detach back to the orchestrator prompt.
  - Press Ctrl-C to stop all services.
"""

import argparse
import os
import shlex
import signal
import selectors
import subprocess
import sys
import termios
import time
import tty
import pty

DETACH_KEY = 0x1D  # Ctrl-]
CTRL_C = 0x03


class ManagedProcess:
    def __init__(self, name, cmd, master_fd, proc):
        self.name = name
        self.cmd = cmd
        self.master_fd = master_fd
        self.proc = proc
        self.display_buffer = b""
        self.log_buffer = b""


def _build_cmd(service, args_str):
    cmd = ["ros2", "run", "confluence", service]
    if args_str:
        cmd.extend(shlex.split(args_str))
    return cmd


def _open_log(path):
    if not path:
        return None
    return open(path, "a", encoding="utf-8")


def _timestamp():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def _write_log(log_file, name, line):
    if log_file is None:
        return
    log_file.write(f"{_timestamp()} [{name}] {line}\n")
    log_file.flush()


def _render_prefixed(name, line):
    return f"[{name}] {line}"


def _print_line(line):
    sys.stdout.write(line + "\n")
    sys.stdout.flush()


def _drain_output(proc, data, attached_name, log_file):
    if not data:
        return

    # Log handling (split by lines)
    proc.log_buffer += data
    while b"\n" in proc.log_buffer:
        raw_line, proc.log_buffer = proc.log_buffer.split(b"\n", 1)
        line = raw_line.decode("utf-8", errors="replace")
        _write_log(log_file, proc.name, line)

    # Display handling
    if attached_name == proc.name:
        os.write(sys.stdout.fileno(), data)
        return

    proc.display_buffer += data
    while b"\n" in proc.display_buffer:
        raw_line, proc.display_buffer = proc.display_buffer.split(b"\n", 1)
        line = raw_line.decode("utf-8", errors="replace")
        _print_line(_render_prefixed(proc.name, line))


def _enter_raw_mode(fd):
    attrs = termios.tcgetattr(fd)
    tty.setraw(fd)
    return attrs


def _restore_term(fd, attrs):
    if attrs is None:
        return
    termios.tcsetattr(fd, termios.TCSADRAIN, attrs)


def _print_help():
    _print_line("Commands: list, <service>, attach <service>, help, quit")
    _print_line("Attach shortcut: type service name (firehose, uniform_pump, inject, fault_detector)")
    _print_line("Detach from service: Ctrl-]")


def main(args=None):
    parser = argparse.ArgumentParser(description="Confluence Orchestrator")
    parser.add_argument("--all", action="store_true", help="Start all services")
    parser.add_argument("--services", nargs="*", default=[], help="Services to start")
    parser.add_argument("--firehose-args", default="", help="Args for firehose")
    parser.add_argument("--uniform-pump-args", default="", help="Args for uniform_pump")
    parser.add_argument("--inject-args", default="", help="Args for inject")
    parser.add_argument("--fault-detector-args", default="", help="Args for fault_detector")
    parser.add_argument("--log-file", default="", help="Write combined output to a log file")

    parsed = parser.parse_args(args=args)

    services = []
    if parsed.all:
        services = ["firehose", "uniform_pump", "inject", "fault_detector"]
    else:
        services = parsed.services or []

    if not services:
        _print_line("No services selected. Use --all or --services.")
        return 1

    args_map = {
        "firehose": parsed.firehose_args,
        "uniform_pump": parsed.uniform_pump_args,
        "inject": parsed.inject_args,
        "fault_detector": parsed.fault_detector_args,
    }

    log_file = _open_log(parsed.log_file)

    selector = selectors.DefaultSelector()
    processes = {}

    def start_service(name):
        cmd = _build_cmd(name, args_map.get(name, ""))
        master_fd, slave_fd = pty.openpty()
        proc = subprocess.Popen(
            cmd,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            start_new_session=True,
            close_fds=True,
        )
        os.close(slave_fd)
        os.set_blocking(master_fd, False)
        managed = ManagedProcess(name, cmd, master_fd, proc)
        processes[name] = managed
        selector.register(master_fd, selectors.EVENT_READ, data=name)
        _print_line(f"Started {name}: {' '.join(cmd)}")

    for svc in services:
        start_service(svc)

    selector.register(sys.stdin, selectors.EVENT_READ, data="stdin")

    attached = None
    raw_attrs = None
    input_buffer = ""

    def shutdown():
        nonlocal raw_attrs
        if raw_attrs is not None:
            _restore_term(sys.stdin.fileno(), raw_attrs)
            raw_attrs = None
        for proc in list(processes.values()):
            try:
                os.killpg(os.getpgid(proc.proc.pid), signal.SIGINT)
            except Exception:
                pass
        if log_file is not None:
            log_file.flush()
            log_file.close()

    def handle_command(line):
        nonlocal attached, raw_attrs
        cmd = line.strip()
        if not cmd:
            return
        if cmd in ("help", "?"):
            _print_help()
            return
        if cmd in ("quit", "exit", "stop"):
            shutdown()
            sys.exit(0)
        if cmd in ("list", "ls"):
            running = ", ".join(processes.keys())
            _print_line(f"Running: {running}")
            return
        if cmd.startswith("attach "):
            _, name = cmd.split(" ", 1)
            name = name.strip()
        else:
            name = cmd
        if name in processes:
            attached = name
            raw_attrs = _enter_raw_mode(sys.stdin.fileno())
            _print_line(f"Attached to {name}. Ctrl-] to detach, Ctrl-C to stop all.")
        else:
            _print_line(f"Unknown command or service: {cmd}")

    try:
        _print_help()
        while processes:
            events = selector.select(timeout=0.1)
            for key, _ in events:
                if key.data == "stdin":
                    if attached is None:
                        line = sys.stdin.readline()
                        if not line:
                            continue
                        handle_command(line)
                    else:
                        data = os.read(sys.stdin.fileno(), 1024)
                        if not data:
                            continue
                        for b in data:
                            if b == DETACH_KEY:
                                _restore_term(sys.stdin.fileno(), raw_attrs)
                                raw_attrs = None
                                _print_line("")
                                _print_line(f"Detached from {attached}.")
                                attached = None
                                break
                            if b == CTRL_C:
                                shutdown()
                                return 0
                            os.write(processes[attached].master_fd, bytes([b]))
                else:
                    name = key.data
                    proc = processes.get(name)
                    if proc is None:
                        continue
                    try:
                        data = os.read(proc.master_fd, 4096)
                    except OSError:
                        data = b""
                    if data:
                        _drain_output(proc, data, attached, log_file)
                    else:
                        selector.unregister(proc.master_fd)
                        os.close(proc.master_fd)
                        rc = proc.proc.poll()
                        if rc is None:
                            rc = proc.proc.wait(timeout=1)
                        _print_line(f"{name} exited with code {rc}")
                        if attached == name:
                            _restore_term(sys.stdin.fileno(), raw_attrs)
                            raw_attrs = None
                            attached = None
                        processes.pop(name, None)
            # Cleanup finished processes
            for name, proc in list(processes.items()):
                if proc.proc.poll() is not None:
                    selector.unregister(proc.master_fd)
                    os.close(proc.master_fd)
                    _print_line(f"{name} exited with code {proc.proc.returncode}")
                    if attached == name:
                        _restore_term(sys.stdin.fileno(), raw_attrs)
                        raw_attrs = None
                        attached = None
                    processes.pop(name, None)
        return 0
    except KeyboardInterrupt:
        shutdown()
        return 0
    finally:
        shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
