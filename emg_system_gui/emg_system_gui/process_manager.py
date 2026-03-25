import os
import signal
import subprocess
import threading
import queue
from typing import Dict, Optional

from .config import ROS_SETUP, WS_SETUP, CONDA_ENV


class ManagedProcess:
    def __init__(self, name: str, popen: subprocess.Popen):
        self.name = name
        self.popen = popen
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
        )
        self.reader_thread.start()

    def _reader_loop(self):
        if self.popen.stdout is None:
            return

        try:
            for line in iter(self.popen.stdout.readline, ''):
                if not line:
                    break
                self.log_queue.put(line.rstrip())
        except Exception as e:
            self.log_queue.put(f"[LOG-ERROR] {self.name}: {e}")

    def is_running(self) -> bool:
        return self.popen.poll() is None


class ProcessManager:
    def __init__(self):
        self.processes: Dict[str, ManagedProcess] = {}

    def _build_command(self, command: str) -> str:
        return (
            "source ~/anaconda3/etc/profile.d/conda.sh && "
            f"conda activate {CONDA_ENV} && "
            f"{ROS_SETUP} && "
            f"{WS_SETUP} && "
            f"exec {command}"
        )

    def start(self, name: str, command: str) -> str:
        if name in self.processes and self.processes[name].is_running():
            return f"[INFO] {name} is already running."

        full_cmd = self._build_command(command)

        proc = subprocess.Popen(
            ["bash", "-ic", full_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,   
        )

        self.processes[name] = ManagedProcess(name, proc)
        return f"[START] {name}"

    def stop(self, name: str) -> str:
        mp: Optional[ManagedProcess] = self.processes.get(name)
        if mp is None:
            return f"[INFO] {name} is not running."

        proc = mp.popen
        if proc.poll() is not None:
            return f"[INFO] {name} already exited."

        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            return f"[STOP] {name}"
        except ProcessLookupError:
            return f"[INFO] {name} already exited."
        except Exception as e:
            return f"[ERROR] failed to stop {name}: {e}"

    def force_kill(self, name: str) -> str:
        mp: Optional[ManagedProcess] = self.processes.get(name)
        if mp is None:
            return f"[INFO] {name} is not running."

        proc = mp.popen
        if proc.poll() is not None:
            return f"[INFO] {name} already exited."

        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
            return f"[KILL] {name}"
        except ProcessLookupError:
            return f"[INFO] {name} already exited."
        except Exception as e:
            return f"[ERROR] failed to kill {name}: {e}"

    def stop_all(self) -> str:
        msgs = []
        for name in list(self.processes.keys()):
            msgs.append(self.stop(name))
        return " | ".join(msgs) if msgs else "[INFO] no processes"

    def get_new_logs(self):
        logs = []
        for name, mp in self.processes.items():
            while not mp.log_queue.empty():
                line = mp.log_queue.get_nowait()
                logs.append(f"[{name}] {line}")
        return logs

    def cleanup_exited(self):
        dead = []
        for name, mp in self.processes.items():
            if not mp.is_running():
                dead.append(name)
        for name in dead:
            pass