import json
import socket
import threading
from urllib.parse import urlencode
from urllib.request import urlopen


class ElegooRobotCar:
    """Client for the custom ELEGOO car firmware. Responses are decoded JSON dicts."""

    def __init__(self, host, port=100, timeout=3):
        self.host, self.port, self.timeout = host, port, timeout
        self.http_base = f"http://{host}"
        self._socket = socket.create_connection((host, port), timeout)
        self._reader = self._socket.makefile("rb")
        self._lock = threading.Lock()

    def command(self, number, d1=None, d2=None, d3=None):
        payload = {"N": number}
        payload.update({key: val for key, val in zip(("D1", "D2", "D3"), (d1, d2, d3)) if val is not None})
        with self._lock:
            self._socket.sendall((json.dumps(payload, separators=(",", ":")) + "\n").encode())
            while True:
                line = self._reader.readline()
                if not line:
                    raise ConnectionError("Arduino bridge connection closed")
                if line.strip() == b"{Heartbeat}":
                    continue
                return json.loads(line)

    def ping(self): return self.command(1)
    def drive(self, left, right):
        if any(isinstance(v, bool) or not isinstance(v, int) or not -255 <= v <= 255 for v in (left, right)):
            raise ValueError("Motor speeds must be integers from -255 to 255")
        return self.command(7, left, right)
    def forward(self, speed=None): return self.command(2, speed)
    def backward(self, speed=None): return self.command(3, speed)
    def turn_left(self, speed=None): return self.command(4, speed)
    def turn_right(self, speed=None): return self.command(5, speed)
    def stop(self): return self.command(6)
    def set_default_speed(self, speed): return self.command(8, speed)
    def read_distance(self): return self.command(10)
    def read_ir(self): return self.command(11)
    def read_imu(self): return self.command(12)
    def read_battery(self): return self.command(13)
    def calibrate_battery(self, voltage): return self.command(14, int(voltage * 100 + 0.5))
    def set_motor_bias(self, left, right): return self.command(15, left, right)
    def set_ir_threshold(self, threshold): return self.command(16, threshold)
    def set_led_color(self, red, green, blue): return self.command(20, red, green, blue)
    def set_led_brightness(self, brightness): return self.command(21, brightness)
    def pan(self, angle): return self.command(30, angle)
    def tilt(self, angle): return self.command(31, angle)
    def all_sensors(self): return self.command(100)
    def state(self): return self.command(101)
    def set_watchdog(self, timeout_ms): return self.command(102, timeout_ms)

    def _http(self, path, params=None, binary=False):
        url = self.http_base + path
        if params is not None:
            url += "?" + urlencode(params)
        with urlopen(url, timeout=self.timeout) as response:
            data = response.read()
        return data if binary else json.loads(data)

    def status(self): return self._http("/status")
    def info(self): return self._http("/info")
    def logs(self): return self._http("/logs")
    def restart_esp32(self): return self._http("/restart")
    def capture(self): return self._http("/capture", binary=True)
    def stream_url(self): return self.http_base + "/stream"
    def set_camera(self, variable, value): return self._http("/control", {"var": variable, "val": value})

    def close(self):
        self._reader.close()
        self._socket.close()

    def __enter__(self): return self
    def __exit__(self, exc_type, exc, tb): self.close()
