#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
Start or restart a single Argo ROS2 node without stopping argo_launch_standard.service.

Default: start the node only if it is not already running (e.g. after killing a
process to run it from the CLI: ars anem). Use --restart to stop and start again.

Uses launch/argo_nodes.yaml for names, executables, and short names (anem, gps).
"""

import argparse
import argcomplete
import os
import shlex
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from argo_node_utils import load_argo_nodes_config  # noqa: E402

LAUNCH_SERVICE = 'argo_launch_standard.service'


def _resolve_ros_setup_bash() -> str:
    distro = os.environ.get('ROS_DISTRO', 'humble')
    candidate = f'/opt/ros/{distro}/setup.bash'
    if os.path.isfile(candidate):
        return candidate
    return '/opt/ros/humble/setup.bash'


def _bash_source_ros_prefix() -> str:
    return f'source {shlex.quote(_resolve_ros_setup_bash())} && '


def _argo_root() -> str:
    return os.path.dirname(_SCRIPT_DIR)


def _list_restartable_names(config: Dict[str, Any]) -> List[str]:
    names: List[str] = []
    groups = config.get('groups', {})
    for group_key in ('physical_robot', 'simulation'):
        for name in groups.get(group_key, []):
            if name and name not in names:
                names.append(name)
    for svc in config.get('services', []):
        name = svc.get('name')
        if name and name not in names:
            names.append(name)
    return sorted(names)


def _resolve_node_query(
    config: Dict[str, Any], query: str
) -> Tuple[str, Dict[str, Any]]:
    """Return ('launch'|'service', config_entry)."""
    q = query.strip()
    if not q:
        raise SystemExit('Node name required (e.g. argo_web_dashboard)')

    if q.endswith('.py'):
        q = q[:-3]

    nodes = config.get('nodes', [])
    services = config.get('services', [])
    by_name = {n['name']: n for n in nodes if n.get('name')}
    service_by_name = {s['name']: s for s in services if s.get('name')}

    def _from_node_cfg(cfg: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
        name = cfg['name']
        if cfg.get('excluded', False):
            if name in service_by_name:
                return 'service', service_by_name[name]
            raise SystemExit(
                f"Node '{name}' is excluded from launch (disabled or not running). "
                "It cannot be restarted with this tool."
            )
        return 'launch', cfg

    if q in by_name:
        return _from_node_cfg(by_name[q])

    if q in service_by_name:
        return 'service', service_by_name[q]

    for cfg in nodes:
        executable = cfg.get('executable', '')
        base = os.path.basename(executable).replace('.py', '')
        if base and (q == base or q == executable):
            return _from_node_cfg(cfg)

    known = ', '.join(_list_restartable_names(config)[:12])
    suffix = '...' if len(_list_restartable_names(config)) > 12 else ''
    raise SystemExit(
        f"Unknown node '{query}'. Examples: {known}{suffix}\n"
        "Use ROS2 node names from launch/argo_nodes.yaml (e.g. argo_web_dashboard)."
    )


def _executable_path(argo_dir: str, cfg: Dict[str, Any]) -> str:
    executable = cfg.get('executable', '')
    if executable.startswith('nodes/'):
        return os.path.join(argo_dir, executable)
    return executable


def _find_pids(argo_dir: str, cfg: Dict[str, Any]) -> List[int]:
    name = cfg.get('name', '')
    if cfg.get('special', False) and 'foxglove' in name:
        pattern = 'foxglove_bridge'
    else:
        path = _executable_path(argo_dir, cfg)
        pattern = path if os.path.isabs(path) else os.path.join(argo_dir, path)

    try:
        result = subprocess.run(
            ['pgrep', '-f', pattern],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []

    pids = []
    self_pid = os.getpid()
    for line in (result.stdout or '').strip().splitlines():
        if not line.strip():
            continue
        try:
            pid = int(line.strip())
        except ValueError:
            continue
        if pid != self_pid:
            pids.append(pid)
    return sorted(set(pids))


def _terminate_pids(pids: List[int], label: str) -> None:
    if not pids:
        print(f"ℹ️  No running process found for {label} (will start fresh)")
        return

    print(f"🛑 Stopping {label} (PID(s): {', '.join(map(str, pids))})...")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    time.sleep(2)
    survivors = [p for p in pids if _pid_alive(p)]
    for pid in survivors:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if survivors:
        time.sleep(0.5)


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def _launch_node(argo_dir: str, cfg: Dict[str, Any]) -> subprocess.Popen:
    name = cfg.get('name', 'node')
    argo_yaml = os.path.join(argo_dir, 'nodes', 'argo.yaml')
    node_args = list(cfg.get('args') or [])

    if cfg.get('special', False):
        executable = cfg.get('executable', '')
        if 'ros2 run' in executable:
            parts = executable.replace('ros2 run', '').strip().split()
            if len(parts) < 2:
                raise SystemExit(f"Invalid special executable for {name}: {executable}")
            package, exec_name = parts[0], parts[1]
            cmd_inner = (
                f'ros2 run {package} {exec_name} '
                f'--ros-args --log-level warn'
            )
        else:
            raise SystemExit(f"Unsupported special node launch for {name}")
    else:
        exe_path = _executable_path(argo_dir, cfg)
        if not os.path.isfile(exe_path):
            raise SystemExit(f"Executable not found: {exe_path}")
        quoted = ' '.join(
            [
                'python3',
                shlex.quote(exe_path),
                *[shlex.quote(a) for a in node_args],
                '--ros-args',
                '--params-file',
                shlex.quote(argo_yaml),
            ]
        )
        cmd_inner = quoted

    cmd = ['bash', '-c', _bash_source_ros_prefix() + cmd_inner]
    print(f"🚀 Starting {name}...")
    return subprocess.Popen(
        cmd,
        cwd=argo_dir,
        stdout=None,
        stderr=None,
        start_new_session=True,
    )


def _systemd_service_active(service_name: str) -> bool:
    result = subprocess.run(
        ['systemctl', 'is-active', '--quiet', service_name],
        capture_output=True,
    )
    return result.returncode == 0


def _ensure_systemd_service(svc_cfg: Dict[str, Any], force_restart: bool) -> int:
    service_name = svc_cfg.get('service_name')
    if not service_name:
        raise SystemExit(f"No service_name for {svc_cfg.get('name')}")

    name = svc_cfg.get('name', service_name)
    if _systemd_service_active(service_name) and not force_restart:
        print(f"ℹ️  {service_name} ({name}) is already active")
        print(f"   Use: ars {name} --restart  to restart the service")
        return 0

    action = (
        'restart'
        if force_restart and _systemd_service_active(service_name)
        else 'start'
    )
    print(f"🔄 {action.capitalize()}ing systemd service {service_name} ({name})...")
    result = subprocess.run(
        ['sudo', 'systemctl', action, service_name],
        timeout=30,
    )
    if result.returncode != 0:
        print(f"❌ systemctl {action} {service_name} failed (exit {result.returncode})")
        return result.returncode
    print(f"✅ {service_name} {action}ed")
    return 0


def _verify_ros_node(name: str, timeout_sec: float = 8.0) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        try:
            result = subprocess.run(
                [
                    'bash',
                    '-c',
                    _bash_source_ros_prefix()
                    + f'timeout 3 ros2 node list 2>/dev/null | grep -q "/{name}"',
                ],
                timeout=6,
            )
            if result.returncode == 0:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.5)
    return False


def start_node(
    node_query: str, skip_verify: bool = False, force_restart: bool = False
) -> int:
    argo_dir = _argo_root()
    config = load_argo_nodes_config(argo_dir)
    kind, cfg = _resolve_node_query(config, node_query)
    name = cfg.get('name', node_query)

    if kind == 'service':
        return _ensure_systemd_service(cfg, force_restart)

    active = subprocess.run(
        ['systemctl', 'is-active', '--quiet', LAUNCH_SERVICE],
        capture_output=True,
    )
    if active.returncode != 0:
        print(
            f"⚠️  {LAUNCH_SERVICE} is not active. "
            "Start the fleet with: al"
        )
        return 1

    pids = _find_pids(argo_dir, cfg)
    if pids and not force_restart:
        print(
            f"ℹ️  {name} already running (PID(s): {', '.join(map(str, pids))})"
        )
        print(f"   Use: ars {node_query} --restart  to stop and start again")
        return 0

    if pids and force_restart:
        _terminate_pids(pids, name)
    else:
        print(f"ℹ️  {name} is not running — starting...")

    proc = _launch_node(argo_dir, cfg)
    time.sleep(1)
    if proc.poll() is not None:
        print(f"❌ {name} exited immediately (code {proc.returncode})")
        print("   Check: journalctl -u argo_launch_standard.service -n 30")
        return 1

    print(f"✅ {name} launched (PID {proc.pid})")

    if not skip_verify and not cfg.get('special', False):
        if _verify_ros_node(name):
            print(f"✅ {name} visible in ros2 node list")
        else:
            print(
                f"⚠️  {name} process running but not yet in ros2 node list "
                "(may still be initializing)"
            )
    return 0


def main() -> None:
    config = load_argo_nodes_config(_argo_root())
    parser = argparse.ArgumentParser(
        description=(
            'Start one Argo node if not running, or restart with --restart. '
            'Does not stop argo_launch_standard.service.'
        ),
        epilog=(
            'Examples: ars anem | ars gps_node | ars argo_web_dashboard --restart\n'
            'Full fleet restart: ars (no arguments)'
        ),
    )
    parser.add_argument(
        'node',
        nargs='?',
        help='Node name or script stem from argo_nodes.yaml (e.g. anem, gps, argo_web_dashboard)',
    )
    parser.add_argument(
        '--restart',
        action='store_true',
        help='Stop the node if running, then start it again',
    )
    parser.add_argument(
        '--no-verify',
        action='store_true',
        help='Skip ros2 node list check after launch',
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List node names and exit',
    )
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.list:
        for name in _list_restartable_names(config):
            print(name)
        return

    if not args.node:
        parser.print_help()
        print("\nFor all nodes: use ars with no arguments (aq && al).")
        sys.exit(2)

    sys.exit(
        start_node(
            args.node,
            skip_verify=args.no_verify,
            force_restart=args.restart,
        )
    )


if __name__ == '__main__':
    main()
