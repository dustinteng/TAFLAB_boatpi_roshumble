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

        # Subscriptions to control commands and GPS
        self.create_subscription(ControlData, '/boatcontrol', self.ctrl_cb, 10)
        self.create_subscription(NavSatFix,     '/gps/fix',     self.gps_cb,  10)

        # Set up the figure with two subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.line1, = self.ax1.plot([], [], 'b-')  # Rudder angle vs time
        self.line2, = self.ax2.plot([], [], 'r-')  # Latitude vs time

        # Animate with caching disabled to avoid unbounded memory use
        self.ani = FuncAnimation(
            self.fig,
            self.update,
            interval=500,
            cache_frame_data=False
        )

        plt.tight_layout()
        plt.show()

    def ctrl_cb(self, msg: ControlData):
        """Callback for /boatcontrol: record rudder angle and timestamp."""
        self.rudders.append(msg.servo_rudder)
        self.times.append(len(self.times))

    def gps_cb(self, msg: NavSatFix):
        """Callback for /gps/fix: record latitude."""
        self.lats.append(msg.latitude)

    def update(self, frame):
        """Called by FuncAnimation to update the plots."""
        # Update rudder-angle plot
        if self.times and self.rudders:
            self.line1.set_data(self.times, self.rudders)
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax1.set_ylabel("Rudder (°)")

        # Update latitude plot
        if self.times and self.lats:
            self.line2.set_data(self.times, self.lats)
            self.ax2.relim()
            self.ax2.autoscale_view()
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
