#Elegoo Python Examples

To run Python example files, in a terminal, type 'python3 <example_code>.py'.

To run Python Elegoo interface interactively, use

from elegoo_robot_car import ElegooRobotCar

```python
with ElegooRobotCar("192.168.4.1") as robot:
    print(robot.ping())
    print(robot.read_distance())
    print(robot.read_ir())
```
