import rclpy
from rclpy.node import Node
from taflab_msgs.msg import ControlData
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Visualizer(Node):
    def __init__(self):
        super().__init__('test_visualizer')
        self.times = []
        self.rudders = []
        self.lats = []

        self.create_subscription(ControlData, '/boatcontrol', self.ctrl_cb, 10)
        self.create_subscription(NavSatFix,     '/gps/fix',     self.gps_cb,  10)

        # set up live plot
        self.fig, (self.ax1, self.ax2) = plt.subplots(2,1)
        self.line1, = self.ax1.plot([], [], 'b-')  # rudder vs time
        self.line2, = self.ax2.plot([], [], 'r-')  # latitude vs time

        self.ani = FuncAnimation(self.fig, self.update, interval=500)
        plt.tight_layout()
        plt.show()

    def ctrl_cb(self, msg):
        self.rudders.append(msg.servo_rudder)
        self.times.append(len(self.times))

    def gps_cb(self, msg):
        self.lats.append(msg.latitude)

    def update(self, _):
        if self.times and self.rudders:
            self.line1.set_data(self.times, self.rudders)
            self.ax1.relim(); self.ax1.autoscale_view()
            self.ax1.set_ylabel("Rudder (°)")
        if self.times and self.lats:
            self.line2.set_data(self.times, self.lats)
            self.ax2.relim(); self.ax2.autoscale_view()
            self.ax2.set_ylabel("Latitude")
        return self.line1, self.line2

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
