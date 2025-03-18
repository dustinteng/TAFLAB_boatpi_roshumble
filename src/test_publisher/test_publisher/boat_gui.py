import sys
import threading
import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, QtCore, QtGui
from taflab_msgs.msg import ControlData  # Custom message with fields: servo_sail, servo_rudder, esc

class BoatControlGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Boat Control GUI")
        self.setGeometry(100, 100, 400, 300)
        self.rudder_angle = 0.0  # in degrees
        self.sail_angle = 0.0    # in degrees
        self.esc_value = 0.0

    def update_display(self, rudder_angle, sail_angle, esc_value):
        self.rudder_angle = rudder_angle
        self.sail_angle = sail_angle
        self.esc_value = esc_value
        self.update()  # Trigger a repaint

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        
        # Center of the widget
        center_x = self.width() // 2
        center_y = self.height() // 2

        # Draw boat body as a rectangle
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 2))
        painter.drawRect(center_x - 50, center_y - 20, 100, 40)

        # Draw rudder (as a line) at the bottom-center of the boat,
        # rotated by the rudder angle.
        painter.save()
        painter.translate(center_x, center_y + 20)  # pivot point at the boat's bottom center
        painter.rotate(self.rudder_angle)  # rotate according to rudder command
        painter.drawLine(0, 0, 0, 30)
        painter.restore()

        # Draw sail (as a line) from the center of the boat upward,
        # rotated by the sail angle.
        painter.save()
        painter.translate(center_x, center_y)
        painter.rotate(self.sail_angle)
        painter.drawLine(0, 0, 0, -60)
        painter.restore()

        # Display text values for debugging
        painter.drawText(10, 20, f"Rudder Angle: {self.rudder_angle:.2f}°")
        painter.drawText(10, 40, f"Sail Angle: {self.sail_angle:.2f}°")
        painter.drawText(10, 60, f"ESC Value: {self.esc_value:.2f}")

class BoatControlNode(Node):
    def __init__(self, gui_callback):
        super().__init__('boat_control_gui_node')
        self.subscription = self.create_subscription(
            ControlData,
            '/boatcontrol',
            self.listener_callback,
            10)
        # The callback provided from the GUI to update the display
        self.gui_callback = gui_callback

    def listener_callback(self, msg):
        # Assume the message fields are:
        #   msg.servo_rudder: rudder angle (degrees)
        #   msg.servo_sail: sail angle (degrees)
        #   msg.esc: ESC value (if needed)
        rudder_angle = msg.servo_rudder
        sail_angle = msg.servo_sail
        esc_value = msg.esc
        # Pass these values to the GUI callback. Use a Qt timer to safely update the GUI.
        QtCore.QTimer.singleShot(0, lambda: self.gui_callback(rudder_angle, sail_angle, esc_value))

def main(args=None):
    rclpy.init(args=args)
    
    # Create the Qt Application
    app = QtWidgets.QApplication(sys.argv)
    gui = BoatControlGUI()
    gui.show()

    # Define a function to update the GUI display.
    def update_gui(rudder_angle, sail_angle, esc_value):
        gui.update_display(rudder_angle, sail_angle, esc_value)

    # Create the ROS2 node for boat control GUI and pass the update callback.
    node = BoatControlNode(update_gui)
    
    # Run the ROS2 spinning loop in a separate thread.
    rclpy_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    rclpy_thread.start()

    # Start the Qt event loop.
    ret = app.exec_()

    # Clean up on exit.
    node.destroy_node()
    rclpy.shutdown()
    rclpy_thread.join()
    sys.exit(ret)

if __name__ == '__main__':
    main()
