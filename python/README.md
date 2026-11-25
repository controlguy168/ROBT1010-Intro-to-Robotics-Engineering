# Python client

Python 3.9+; no MATLAB or runtime dependencies. In a terminal in this
folder, install with `python -m pip install .`. Connect the computer
to the car Wi-Fi. 

```python
from elegoo_robot_car import ElegooRobotCar

with ElegooRobotCar("192.168.4.1") as robot:
    print(robot.ping())
    print(robot.read_distance())
    print(robot.read_ir())
    with open("photo.jpg", "wb") as photo:
        photo.write(robot.capture())
    print(robot.stream_url())
```

Methods return decoded JSON dictionaries from the firmware; `capture()` returns JPEG bytes and `stream_url()` returns the MJPEG URL. The context manager closes the TCP connection; it does not stop the motors. Call `robot.stop()` when finished driving. The Arduino watchdog can stop motors after communication is lost; configure it with `set_watchdog(milliseconds)`.

| Method | Parameters | Purpose |
|---|---|---|
| `ping()` | — | Check Arduino connection |
| `drive(left, right)` | integer speeds -255 to 255 | Set left/right speeds |
| `forward(speed)`, `backward(speed)`, `turn_left(speed)`, `turn_right(speed)` | speed 0–255, optional | Move with specified or default speed |
| `stop()` | — | Stop motors |
| `set_default_speed(speed)` | 0–255 | Set default speed |
| `read_distance()`, `read_ir()`, `read_imu()`, `read_battery()` | — | Read sensor data |
| `calibrate_battery(voltage)` | measured volts | Calibrate battery reading |
| `set_motor_bias(left, right)` | percentages 80–120 | Set motor bias |
| `set_ir_threshold(threshold)` | 0–1023 | Set IR threshold |
| `set_led_color(red, green, blue)` | each 0–255 | Set LED color |
| `set_led_brightness(brightness)` | 0–255 | Set LED brightness |
| `pan(angle)`, `tilt(angle)` | 0–180 degrees | Move servos |
| `all_sensors()`, `state()` | — | Read all sensors or Arduino state |
| `set_watchdog(timeout_ms)` | milliseconds; 0 disables | Configure Arduino motor watchdog |
| `status()`, `info()`, `logs()` | — | Read ESP32 HTTP endpoints |
| `restart_esp32()` | — | Restart ESP32 |
| `capture()` | — | Get JPEG bytes |
| `stream_url()` | — | Get MJPEG stream URL |
| `set_camera(variable, value)` | camera setting and value | Change camera setting |
| `command(number, d1=None, d2=None, d3=None)` | command ID and optional parameters | Send a raw Arduino command |
| `close()` | — | Close TCP connection |


