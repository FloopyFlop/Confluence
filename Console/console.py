#!/usr/bin/env python3
"""
Confluence Remote Console

Connects to a running Confluence orchestrator over TCP to stream logs and send
commands (fault injection, parameter updates, topic watch/publish, and probes).

Dependencies:
- Python 3.8+
- No ROS2 required
- Optional: rich (for formatted output)

Usage examples:
  python3 Console/console.py --host 192.168.1.50 --port 9000
  uv run --with rich Console/console.py --host 192.168.1.50 --port 9000

Commands (type at the prompt):
  list
  probe                      (request fault-detector probe diagnostics)
  fault <index>              (1-4)
  clear                      (clears forced fault; restore params using detector defaults)
  clear no-restore           (clear without restore)
  clear now                  (clear and restore immediately, even if armed)
  clear now no-restore       (force-clear only)
  inject PARAM=VALUE ...
  watch <topic> <type>
  unwatch <topic>
  pub <topic> <type> <json>
  quit
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading

try:
    from rich import box
    from rich.console import Console as RichConsole
    from rich.panel import Panel
    from rich.pretty import Pretty
    from rich.table import Table
    from rich.text import Text

    RICH_AVAILABLE = True
except Exception:
    RICH_AVAILABLE = False


PROMPT = "console> "
PRINT_LOCK = threading.Lock()
RICH_ENABLED = False
RCONSOLE = None


def _set_rich(enabled: bool):
    global RICH_ENABLED, RCONSOLE
    RICH_ENABLED = bool(enabled and RICH_AVAILABLE)
    if RICH_ENABLED and RCONSOLE is None:
        RCONSOLE = RichConsole(highlight=False, soft_wrap=True)


def _to_compact_json(value, limit=320):
    try:
        raw = json.dumps(value, separators=(",", ":"), ensure_ascii=False)
    except Exception:
        raw = str(value)
    if len(raw) > limit:
        return raw[:limit] + "..."
    return raw


def _emit(line, *, async_mode=False, style=None):
    with PRINT_LOCK:
        if async_mode:
            # Keep interactive input stable while background lines arrive.
            sys.stdout.write("\r\033[2K")

        if RICH_ENABLED and RCONSOLE is not None:
            if hasattr(line, "__rich_console__"):
                RCONSOLE.print(line)
            else:
                if style:
                    RCONSOLE.print(str(line), style=style, markup=False, highlight=False)
                else:
                    RCONSOLE.print(str(line), markup=False, highlight=False)
        else:
            sys.stdout.write(str(line) + "\n")

        if async_mode:
            sys.stdout.write(PROMPT)
            sys.stdout.flush()
        elif not (RICH_ENABLED and RCONSOLE is not None):
            sys.stdout.flush()


def _print(line: str):
    _emit(line, async_mode=False)


def _print_async(line, style=None):
    _emit(line, async_mode=True, style=style)


def _render_help_rich():
    table = Table(title="Confluence Remote Console Commands", box=box.SIMPLE_HEAVY)
    table.add_column("Command", style="cyan", no_wrap=True)
    table.add_column("Description", style="white")
    table.add_row("list", "List currently running orchestrator services")
    table.add_row("probe", "Request a detector integration probe")
    table.add_row("fault <index>", "Inject forced fault (1-4)")
    table.add_row("clear", "Clear forced fault with default restore behavior")
    table.add_row("clear no-restore", "Clear forced fault without restore")
    table.add_row("clear now", "Clear and request immediate restore")
    table.add_row("clear now no-restore", "Clear only, no restore")
    table.add_row("inject PARAM=VALUE ...", "Send px4_injector set_params command")
    table.add_row("watch <topic> <type>", "Subscribe server-side and stream topic messages")
    table.add_row("unwatch <topic>", "Stop server-side subscription")
    table.add_row("pub <topic> <type> <json>", "Publish a one-off message")
    table.add_row("quit", "Exit client")
    return table


def _print_help():
    if RICH_ENABLED:
        _print(_render_help_rich())
        return

    _print("Commands:")
    _print("  list")
    _print("  probe")
    _print("  fault <index>")
    _print("  clear")
    _print("  clear no-restore")
    _print("  clear now")
    _print("  clear now no-restore")
    _print("  inject PARAM=VALUE ...")
    _print("  watch <topic> <type>")
    _print("  unwatch <topic>")
    _print("  pub <topic> <type> <json>")
    _print("  quit")


def _parse_command(line: str):
    line = line.strip()
    if not line:
        return None
    if line == "help":
        return {"cmd": "help"}

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


def _print_services(services):
    if RICH_ENABLED:
        table = Table(title="Running Services", box=box.SIMPLE)
        table.add_column("Service", style="cyan")
        if services:
            for service in services:
                table.add_row(str(service))
        else:
            table.add_row("(none)")
        _print_async(table)
        return

    _print_async(f"Services: {', '.join(services)}")


def _print_event(msg_type, payload):
    if RICH_ENABLED:
        style_map = {
            "info": "cyan",
            "ack": "green",
            "error": "red",
            "event": "magenta",
        }
        border = style_map.get(msg_type, "white")
        panel = Panel(
            Pretty(payload, expand_all=False),
            title=f"{msg_type.upper()}",
            border_style=border,
            box=box.ROUNDED,
        )
        _print_async(panel)
        return

    _print_async(f"{msg_type.upper()}: {payload}")


def _print_topic(topic, message):
    if RICH_ENABLED:
        renderable = Pretty(message, expand_all=False)
        panel = Panel(renderable, title=f"TOPIC {topic}", border_style="bright_blue", box=box.SQUARE)
        _print_async(panel)
        return

    _print_async(f"[{topic}] {_to_compact_json(message)}")


def _print_log(service, text):
    if RICH_ENABLED:
        line = Text()
        line.append(f"[{service}] ", style="cyan")
        line.append(str(text), style="bright_black")
        _print_async(line)
        return

    _print_async(f"[{service}] {text}")


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
                _print_log(service, text)
            elif msg_type == "services":
                _print_services(payload.get("services", []))
            elif msg_type in ("info", "ack", "error", "event"):
                _print_event(msg_type, payload)
            elif msg_type == "topic":
                topic = payload.get("topic", "?")
                message = payload.get("message")
                _print_topic(topic, message)
            else:
                _print_async(_to_compact_json(payload))


def main():
    parser = argparse.ArgumentParser(description="Confluence Remote Console")
    parser.add_argument("--host", default="127.0.0.1", help="Drone/orchestrator IP")
    parser.add_argument("--port", type=int, default=9000, help="Orchestrator console port")
    parser.add_argument("--plain", action="store_true", help="Disable Rich formatting output")
    parser.add_argument("--rich", action="store_true", help="Force Rich formatting output")
    args = parser.parse_args()

    if args.plain and args.rich:
        _print("ERROR: choose either --plain or --rich, not both")
        return 2

    if args.rich and not RICH_AVAILABLE:
        _print("ERROR: rich is not installed. Try: uv run --with rich Console/console.py --host <IP> --port 9000")
        return 2

    if args.rich:
        _set_rich(True)
    elif args.plain:
        _set_rich(False)
    else:
        _set_rich(RICH_AVAILABLE)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))

    reader_thread = threading.Thread(target=_reader, args=(sock,), daemon=True)
    reader_thread.start()

    mode = "rich" if RICH_ENABLED else "plain"
    _print(f"Connected ({mode} mode). Type 'help' for commands.")
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

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
