# voltage
# current draw
# SOC
# power supply health
# battery temperature
# low battery warning & critical levels
# battery discharge rate and predicted remaining time

from rclpy.node import Node
import math

from sensor_msgs.msg import BatteryState


class PWRMonitor:
    def __init__(self, node: Node, min_voltage = 10.5, max_voltage = 14.1, max_current = 20, timeout_threshold = 1.0):
        self.node = node

        self.min_voltage = min_voltage
        self.max_voltage = max_voltage
        self.max_current = max_current
        self.timeout_threshold = timeout_threshold

        self.battery_state = BatteryState()

        self.nominal = False
        self.overvoltage = False
        self.undervoltage = True
        self.overcurrent = False
        self.last_update = None
        self.stale = None

        self.battery_sub = self.node.create_subscription(BatteryState, "/drone/fc_interface/battery", self.battery_cb, 10)

    def battery_cb(self, msg: BatteryState):
        self.stale = self.is_stale()
        self.last_update = self.node.get_clock().now()

        self.battery_state = msg


    def is_stale(self):
        if self.last_update is None:
            return True
        age = (self.node.get_clock().now().nanoseconds - self.last_update.nanoseconds) / 1e9
        return age > self.timeout_threshold
            

    def update(self):
    
        self.overvoltage = self.battery_state.voltage > self.max_voltage
        self.undervoltage = self.battery_state.voltage < self.min_voltage
        self.stale = False
        self.overcurrent = not math.isnan(self.battery_state.current) and self.battery_state.current > self.max_current


        self.nominal = not(self.overvoltage or self.undervoltage or self.overcurrent or self.stale)
        status = "NOMINAL" if self.nominal else "CRITICAL" 
        return {
            "status": status,
            "PWR_overvoltage": self.overvoltage,
            "PWR_undervoltage": self.undervoltage,
            "PWR_overcurrent": self.overcurrent,
            "PWR_stale": self.stale
        }