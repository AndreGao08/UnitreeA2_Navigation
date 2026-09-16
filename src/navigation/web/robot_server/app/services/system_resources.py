import os
import shutil
import socket
import threading
import time

from robot_server.app.paths import WORKSPACE_ROOT


class SystemResourceMonitor:
    """Low-overhead Linux host metrics sampled by the Web status endpoint."""

    def __init__(self):
        self._lock = threading.Lock()
        self._last_cpu = None
        self._last_network = None

    def snapshot(self):
        with self._lock:
            now = time.monotonic()
            return {
                "cpu": self._cpu_snapshot(),
                "memory": self._memory_snapshot(),
                "disk": self._disk_snapshot(),
                "load": self._load_snapshot(),
                "swap": self._swap_snapshot(),
                "network": self._network_snapshot(now),
                "uptime_seconds": self._uptime_seconds(),
                "sampled_at": time.time(),
            }

    def _cpu_snapshot(self):
        try:
            with open("/proc/stat", "r", encoding="utf-8") as stream:
                values = [int(value) for value in stream.readline().split()[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            total = sum(values)
        except (OSError, ValueError, IndexError):
            return {"percent": None, "cores": os.cpu_count() or 1}
        percent = None
        if self._last_cpu is not None:
            previous_total, previous_idle = self._last_cpu
            delta_total = total - previous_total
            delta_idle = idle - previous_idle
            if delta_total > 0:
                percent = round(max(0.0, min(100.0, 100.0 * (delta_total - delta_idle) / delta_total)), 1)
        self._last_cpu = (total, idle)
        return {"percent": percent, "cores": os.cpu_count() or 1}

    @staticmethod
    def _meminfo():
        result = {}
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as stream:
                for line in stream:
                    key, value = line.split(":", 1)
                    result[key] = int(value.strip().split()[0]) * 1024
        except (OSError, ValueError, IndexError):
            return {}
        return result

    def _memory_snapshot(self):
        info = self._meminfo()
        total = info.get("MemTotal", 0)
        available = info.get("MemAvailable", 0)
        used = max(0, total - available)
        return self._usage(used, total)

    def _swap_snapshot(self):
        info = self._meminfo()
        total = info.get("SwapTotal", 0)
        free = info.get("SwapFree", 0)
        return self._usage(max(0, total - free), total)

    @staticmethod
    def _usage(used, total):
        return {
            "used_bytes": used,
            "total_bytes": total,
            "percent": round(used * 100.0 / total, 1) if total > 0 else 0.0,
        }

    @staticmethod
    def _disk_snapshot():
        try:
            usage = shutil.disk_usage(WORKSPACE_ROOT)
            result = SystemResourceMonitor._usage(usage.used, usage.total)
            result.update({"available_bytes": usage.free, "path": str(WORKSPACE_ROOT)})
            return result
        except OSError:
            return {"used_bytes": 0, "total_bytes": 0, "percent": None, "path": str(WORKSPACE_ROOT)}

    @staticmethod
    def _load_snapshot():
        try:
            one, five, fifteen = os.getloadavg()
        except OSError:
            one = five = fifteen = 0.0
        return {
            "one": round(one, 2), "five": round(five, 2),
            "fifteen": round(fifteen, 2), "cores": os.cpu_count() or 1,
        }

    @staticmethod
    def _network_interface():
        configured = os.getenv("ROBOT_NETWORK_INTERFACE", "").strip()
        if configured and os.path.isdir(f"/sys/class/net/{configured}"):
            return configured
        try:
            with open("/proc/net/route", "r", encoding="utf-8") as stream:
                for line in stream.readlines()[1:]:
                    fields = line.split()
                    if len(fields) > 3 and fields[1] == "00000000" and int(fields[3], 16) & 2:
                        return fields[0]
        except (OSError, ValueError):
            pass
        try:
            names = os.listdir("/sys/class/net")
            physical_prefixes = ("en", "eth", "wl", "ww")
            candidates = [name for name in names if name.startswith(physical_prefixes)]
            candidates.extend(name for name in names if name != "lo" and name not in candidates)
            for name in candidates:
                try:
                    with open(f"/sys/class/net/{name}/operstate", "r", encoding="utf-8") as stream:
                        if stream.read().strip() == "up":
                            return name
                except OSError:
                    continue
            return candidates[0] if candidates else None
        except OSError:
            return None

    def _network_snapshot(self, now):
        interface = self._network_interface()
        received = transmitted = 0
        online = False
        if interface:
            try:
                base = f"/sys/class/net/{interface}"
                with open(f"{base}/statistics/rx_bytes", "r", encoding="utf-8") as stream:
                    received = int(stream.read())
                with open(f"{base}/statistics/tx_bytes", "r", encoding="utf-8") as stream:
                    transmitted = int(stream.read())
                with open(f"{base}/operstate", "r", encoding="utf-8") as stream:
                    online = stream.read().strip() == "up"
            except (OSError, ValueError):
                pass
        receive_rate = transmit_rate = 0.0
        if self._last_network and self._last_network[0] == interface:
            _, previous_time, previous_received, previous_transmitted = self._last_network
            elapsed = now - previous_time
            if elapsed > 0:
                receive_rate = max(0.0, (received - previous_received) / elapsed)
                transmit_rate = max(0.0, (transmitted - previous_transmitted) / elapsed)
        self._last_network = (interface, now, received, transmitted)
        return {
            "interface": interface, "online": online,
            "receive_bytes_per_second": round(receive_rate, 1),
            "transmit_bytes_per_second": round(transmit_rate, 1),
            "received_bytes": received, "transmitted_bytes": transmitted,
            "hostname": socket.gethostname(),
        }

    @staticmethod
    def _uptime_seconds():
        try:
            with open("/proc/uptime", "r", encoding="utf-8") as stream:
                return int(float(stream.read().split()[0]))
        except (OSError, ValueError, IndexError):
            return 0


system_resource_monitor = SystemResourceMonitor()
