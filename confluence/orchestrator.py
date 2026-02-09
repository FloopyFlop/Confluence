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

  # Remote console bridge is enabled by default on 0.0.0.0:9000.
  # Disable it explicitly:
  ros2 run confluence orchestrator --all --console-port 0

Interactive commands (type in the orchestrator prompt):
  list                    Show running services
  firehose                Attach to firehose output/IO
  uniform_pump            Attach to uniform_pump output/IO
  inject                  Attach to inject output/IO
  fault_detector          Attach to fault_detector output/IO
  attach <name>           Same as typing the service name
  help                    Show help
  quit                    Stop all services and exit

Remote console commands (via TCP):
  list
  fault <1-4>
  clear
  inject PARAM=VALUE ...
  watch <topic> <type>
  unwatch <topic>
  pub <topic> <type> <json>

While attached:
  - All keystrokes go to the attached service.
  - Press Ctrl-] to detach back to the orchestrator prompt.
  - Press Ctrl-C to stop all services.
"""

import argparse
import json
import os
import queue
import socket
import shlex
import signal
import selectors
import subprocess
import sys
import termios
import threading
import time
import tty
import pty

DETACH_KEY = 0x1D  # Ctrl-]
CTRL_C = 0x03

try:
    import rclpy
    from std_msgs.msg import String
    RCLPY_AVAILABLE = True
except Exception:
    RCLPY_AVAILABLE = False

try:
    from rosidl_runtime_py import message_to_ordereddict
    ROSIDL_AVAILABLE = True
except Exception:
    ROSIDL_AVAILABLE = False


def _load_env_file(path):
    if not path:
        return {}
    if not os.path.exists(path):
        return {}

    values = {}
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for raw in handle:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export ") :].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                if key:
                    values[key] = value
    except Exception:
        return {}
    return values


def _env_or_default(explicit, env_values, key, default):
    if explicit is not None:
        return explicit
    return env_values.get(key, default)


def _env_int(explicit, env_values, key, default):
    if explicit is not None:
        return int(explicit)
    raw = env_values.get(key)
    if raw is None:
        return default
    try:
        return int(raw)
    except Exception:
        return default


class ManagedProcess:
    def __init__(self, name, cmd, master_fd, proc):
        self.name = name
        self.cmd = cmd
        self.master_fd = master_fd
        self.proc = proc
        self.display_buffer = b""
        self.log_buffer = b""


class ConsoleClient:
    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = addr
        self.lock = threading.Lock()


class ConsoleServer:
    def __init__(self, host, port, command_queue):
        self.host = host
        self.port = port
        self.command_queue = command_queue
        self.clients = []
        self.clients_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._server_thread = None
        self._sock = None

    def start(self):
        self._server_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._server_thread.start()

    def stop(self):
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.sock.close()
                except Exception:
                    pass
            self.clients = []

    def _accept_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(5)
        sock.settimeout(1.0)
        self._sock = sock
        while not self._stop_event.is_set():
            try:
                client_sock, addr = sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            client = ConsoleClient(client_sock, addr)
            with self.clients_lock:
                self.clients.append(client)
            self.send(client, {"type": "info", "message": "connected"})
            thread = threading.Thread(target=self._client_loop, args=(client,), daemon=True)
            thread.start()

    def _client_loop(self, client):
        try:
            with client.sock.makefile("r", encoding="utf-8", errors="replace") as reader:
                for line in reader:
                    if self._stop_event.is_set():
                        break
                    self.command_queue.put((client, line.strip()))
        finally:
            self._remove_client(client)

    def _remove_client(self, client):
        with self.clients_lock:
            self.clients = [c for c in self.clients if c is not client]
        try:
            client.sock.close()
        except Exception:
            pass

    def broadcast(self, payload):
        line = json.dumps(payload) + "\n"
        data = line.encode("utf-8")
        with self.clients_lock:
            clients = list(self.clients)
        for client in clients:
            try:
                with client.lock:
                    client.sock.sendall(data)
            except Exception:
                self._remove_client(client)

    def send(self, client, payload):
        line = json.dumps(payload) + "\n"
        data = line.encode("utf-8")
        try:
            with client.lock:
                client.sock.sendall(data)
        except Exception:
            self._remove_client(client)


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


def _drain_output(proc, data, attached_name, log_file, console_server=None):
    if not data:
        return

    # Log handling (split by lines)
    proc.log_buffer += data
    while b"\n" in proc.log_buffer:
        raw_line, proc.log_buffer = proc.log_buffer.split(b"\n", 1)
        line = raw_line.decode("utf-8", errors="replace")
        _write_log(log_file, proc.name, line)
        if console_server is not None:
            console_server.broadcast({"type": "log", "service": proc.name, "line": line})

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


def _import_msg_type(type_str):
    if not type_str:
        raise ValueError("type_str required")
    normalized = type_str.replace("/", ".")
    if ".msg." not in normalized:
        if "." in normalized:
            pkg, name = normalized.rsplit(".", 1)
            normalized = f"{pkg}.msg.{name}"
        else:
            raise ValueError("type_str must be like std_msgs/msg/String")
    module_name, class_name = normalized.rsplit(".", 1)
    module = __import__(module_name, fromlist=[class_name])
    return getattr(module, class_name)


def _message_to_dict(msg):
    if ROSIDL_AVAILABLE:
        try:
            return message_to_ordereddict(msg)
        except Exception:
            pass
    return str(msg)


def _apply_fields(msg, data):
    for key, value in data.items():
        if isinstance(value, dict):
            sub = getattr(msg, key)
            _apply_fields(sub, value)
        else:
            setattr(msg, key, value)


def main(args=None):
    parser = argparse.ArgumentParser(description="Confluence Orchestrator")
    parser.add_argument("--all", action="store_true", help="Start all services")
    parser.add_argument("--services", nargs="*", default=[], help="Services to start")
    parser.add_argument("--env-file", default=".drone-env", help="Path to .drone-env defaults file")
    parser.add_argument("--firehose-args", default=None, help="Args for firehose")
    parser.add_argument("--uniform-pump-args", default=None, help="Args for uniform_pump")
    parser.add_argument("--inject-args", default=None, help="Args for inject")
    parser.add_argument("--fault-detector-args", default=None, help="Args for fault_detector")
    parser.add_argument("--log-file", default="", help="Write combined output to a log file")
    parser.add_argument("--console-host", default=None, help="Console bind host")
    parser.add_argument(
        "--console-port",
        type=int,
        default=None,
        help="Console TCP port (default 9000, 0 disables)",
    )

    parsed = parser.parse_args(args=args)
    env_values = _load_env_file(parsed.env_file)

    firehose_args = _env_or_default(
        parsed.firehose_args, env_values, "CONFLUENCE_FIREHOSE_ARGS", ""
    )
    uniform_pump_args = _env_or_default(
        parsed.uniform_pump_args, env_values, "CONFLUENCE_UNIFORM_PUMP_ARGS", ""
    )
    inject_args = _env_or_default(
        parsed.inject_args, env_values, "CONFLUENCE_INJECT_ARGS", ""
    )
    fault_detector_args = _env_or_default(
        parsed.fault_detector_args, env_values, "CONFLUENCE_FAULT_DETECTOR_ARGS", ""
    )
    console_host = _env_or_default(
        parsed.console_host, env_values, "CONFLUENCE_CONSOLE_HOST", "0.0.0.0"
    )
    console_port = _env_int(
        parsed.console_port, env_values, "CONFLUENCE_CONSOLE_PORT", 9000
    )

    services = []
    if parsed.all:
        services = ["firehose", "uniform_pump", "inject", "fault_detector"]
    else:
        services = parsed.services or []

    if not services:
        _print_line("No services selected. Use --all or --services.")
        return 1

    args_map = {
        "firehose": firehose_args,
        "uniform_pump": uniform_pump_args,
        "inject": inject_args,
        "fault_detector": fault_detector_args,
    }

    log_file = _open_log(parsed.log_file)

    command_queue = queue.Queue()
    console_server = None
    if console_port:
        console_server = ConsoleServer(console_host, console_port, command_queue)
        console_server.start()
        _print_line(f"Console server listening on {console_host}:{console_port}")

    ros_node = None
    fault_pub = None
    inject_pub = None
    topic_subs = {}
    topic_types = {}
    topic_pubs = {}
    if console_server is not None:
        if RCLPY_AVAILABLE:
            rclpy.init(args=None)
            ros_node = rclpy.create_node("confluence_orchestrator_console")
            fault_pub = ros_node.create_publisher(String, "fault_detector/command", 10)
            inject_pub = ros_node.create_publisher(String, "px4_injector/command", 10)
        else:
            _print_line("Warning: rclpy not available; console commands disabled.")

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
        if console_server is not None:
            console_server.broadcast({"type": "event", "service": name, "event": "started"})

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
        if ros_node is not None:
            for sub in list(topic_subs.values()):
                try:
                    ros_node.destroy_subscription(sub)
                except Exception:
                    pass
        if log_file is not None:
            log_file.flush()
            log_file.close()
        if console_server is not None:
            console_server.stop()
        if ros_node is not None:
            try:
                ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

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

    def parse_console_command(line):
        if not line:
            return None
        line = line.strip()
        if not line:
            return None
        if line.startswith("{"):
            try:
                return json.loads(line)
            except Exception:
                return {"cmd": "invalid", "raw": line}
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
        if cmd in ("set_params", "inject"):
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
        if cmd in ("publish", "pub"):
            split = line.split(maxsplit=3)
            if len(split) < 4:
                return {"cmd": "publish"}
            _, topic, type_str, raw_json = split
            try:
                data = json.loads(raw_json)
            except Exception:
                data = raw_json
            return {"cmd": "publish", "topic": topic, "type": type_str, "data": data}
        return {"cmd": cmd}

    def handle_console_command(client, payload):
        if console_server is None:
            return
        cmd = payload.get("cmd", "")
        if cmd == "invalid":
            console_server.send(client, {"type": "error", "message": "Invalid JSON"})
            return
        if cmd == "list":
            console_server.send(client, {"type": "services", "services": list(processes.keys())})
            return
        if cmd == "fault":
            if fault_pub is None:
                console_server.send(client, {"type": "error", "message": "Fault publisher not available"})
                return
            msg = {"action": "inject_fault"}
            if "fault_index" in payload:
                msg["fault_index"] = payload["fault_index"]
            if "fault_label" in payload:
                msg["fault_label"] = payload["fault_label"]
            if "params" in payload:
                msg["params"] = payload["params"]
            if "apply_inject" in payload:
                msg["apply_inject"] = bool(payload["apply_inject"])
            ros_msg = String()
            ros_msg.data = json.dumps(msg)
            fault_pub.publish(ros_msg)
            console_server.send(client, {"type": "ack", "message": "fault injected"})
            return
        if cmd == "clear_fault":
            if fault_pub is None:
                console_server.send(client, {"type": "error", "message": "Fault publisher not available"})
                return
            ros_msg = String()
            clear_cmd = {"action": "clear_fault"}
            if "restore" in payload:
                clear_cmd["restore"] = bool(payload.get("restore"))
            restore_params = payload.get("restore_params")
            if isinstance(restore_params, dict):
                clear_cmd["restore_params"] = restore_params
            ros_msg.data = json.dumps(clear_cmd)
            fault_pub.publish(ros_msg)
            console_server.send(client, {"type": "ack", "message": "fault cleared"})
            return
        if cmd == "set_params":
            if inject_pub is None:
                console_server.send(client, {"type": "error", "message": "Inject publisher not available"})
                return
            params = payload.get("params", {})
            ros_msg = String()
            ros_msg.data = json.dumps({"action": "set_params", "params": params})
            inject_pub.publish(ros_msg)
            console_server.send(client, {"type": "ack", "message": "inject sent"})
            return
        if cmd == "motortest":
            console_server.send(client, {"type": "error", "message": "Usage: motortest on|off"})
            return
        if cmd == "watch":
            if ros_node is None:
                console_server.send(client, {"type": "error", "message": "ROS node not available"})
                return
            topic = payload.get("topic")
            type_str = payload.get("type")
            if not topic or not type_str:
                console_server.send(client, {"type": "error", "message": "watch requires topic and type"})
                return
            try:
                msg_type = _import_msg_type(type_str)
            except Exception as exc:
                console_server.send(client, {"type": "error", "message": f"invalid type: {exc}"})
                return
            if topic in topic_subs:
                try:
                    ros_node.destroy_subscription(topic_subs[topic])
                except Exception:
                    pass
            def _cb(msg, t=topic):
                payload = {
                    "type": "topic",
                    "topic": t,
                    "message": _message_to_dict(msg),
                }
                console_server.broadcast(payload)
            sub = ros_node.create_subscription(msg_type, topic, _cb, 10)
            topic_subs[topic] = sub
            topic_types[topic] = type_str
            console_server.send(client, {"type": "ack", "message": f"watching {topic} ({type_str})"})
            return
        if cmd == "unwatch":
            if ros_node is None:
                console_server.send(client, {"type": "error", "message": "ROS node not available"})
                return
            topic = payload.get("topic")
            if not topic:
                console_server.send(client, {"type": "error", "message": "unwatch requires topic"})
                return
            sub = topic_subs.pop(topic, None)
            topic_types.pop(topic, None)
            if sub is not None:
                try:
                    ros_node.destroy_subscription(sub)
                except Exception:
                    pass
            console_server.send(client, {"type": "ack", "message": f"unwatched {topic}"})
            return
        if cmd == "publish":
            if ros_node is None:
                console_server.send(client, {"type": "error", "message": "ROS node not available"})
                return
            topic = payload.get("topic")
            type_str = payload.get("type")
            data = payload.get("data")
            if not topic or not type_str or data is None:
                console_server.send(client, {"type": "error", "message": "publish requires topic, type, data"})
                return
            try:
                msg_type = _import_msg_type(type_str)
            except Exception as exc:
                console_server.send(client, {"type": "error", "message": f"invalid type: {exc}"})
                return
            pub = topic_pubs.get((topic, type_str))
            if pub is None:
                pub = ros_node.create_publisher(msg_type, topic, 10)
                topic_pubs[(topic, type_str)] = pub
            try:
                msg = msg_type()
                if isinstance(data, dict):
                    _apply_fields(msg, data)
                else:
                    msg.data = data
                pub.publish(msg)
                console_server.send(client, {"type": "ack", "message": f"published to {topic}"})
            except Exception as exc:
                console_server.send(client, {"type": "error", "message": f"publish failed: {exc}"})
            return
        console_server.send(client, {"type": "error", "message": f"Unknown command: {cmd}"})

    try:
        _print_help()
        while processes:
            if console_server is not None:
                while True:
                    try:
                        client, line = command_queue.get_nowait()
                    except queue.Empty:
                        break
                    payload = parse_console_command(line)
                    if payload is None:
                        continue
                    handle_console_command(client, payload)
            if ros_node is not None:
                try:
                    rclpy.spin_once(ros_node, timeout_sec=0.0)
                except Exception:
                    pass
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
                        _drain_output(proc, data, attached, log_file, console_server)
                    else:
                        selector.unregister(proc.master_fd)
                        os.close(proc.master_fd)
                        rc = proc.proc.poll()
                        if rc is None:
                            rc = proc.proc.wait(timeout=1)
                        _print_line(f"{name} exited with code {rc}")
                        if console_server is not None:
                            console_server.broadcast({"type": "event", "service": name, "event": "exited", "code": rc})
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
                    if console_server is not None:
                        console_server.broadcast({"type": "event", "service": name, "event": "exited", "code": proc.proc.returncode})
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
