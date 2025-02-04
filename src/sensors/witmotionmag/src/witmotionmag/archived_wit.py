import rclpy
from rclpy.node import Node
from serial import Serial
from std_msgs.msg import Float32
from sensor_msgs.msg import MagneticField  # <-- Import MagneticField message
import witmotion
import time
import datetime
import platform
import struct
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver
import math

_writeF = None
_IsWriteF = False

class WitMotionMagNode(Node):
    
    def __init__(self):
        super().__init__('witmotionmag_node')
        
        self.angle_publisher = self.create_publisher(Float32, 'witmotion_angle', 10)
        
        # NEW: Publisher for MagneticField
        self.mag_publisher = self.create_publisher(MagneticField, 'magnetic_field', 10)

        self.timer = self.create_timer(0.1, self.updateData)  # Publish every 0.1 seconds
        
        # 初始化一个设备模型 (Initialize a device model)
        self.device = deviceModel.DeviceModel(
            "我的JY901",
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0"
        )
        if platform.system().lower() == 'linux':
            self.device.serialConfig.portName = "/dev/ttyMag"  # 设置串口 (Set serial port)
        else:
            self.device.serialConfig.portName = "/dev/ttyUSB0" # Example for another OS
        
        self.device.serialConfig.baud = 9600  # Set baud rate
        self.device.openDevice()              # Open serial port
        self.readConfig(self.device)          # Read configuration info

    def readConfig(self, device):
        """
        Reads configuration info as an example.
        """
        tVals = device.readReg(0x02, 3)  # Read data content, return rate, communication rate
        if len(tVals) > 0:
            print("Returns:", str(tVals))
        else:
            print("No Return")

        tVals = device.readReg(0x23, 2)  # Read installation direction and algorithm
        if len(tVals) > 0:
            print("返回结果：", str(tVals))
        else:
            print("Returns:")

    def setConfig(self, device):
        """
        Example of setting configuration info on the device.
        """
        device.unlock()
        time.sleep(0.1)
        device.writeReg(0x03, 6)  # Return rate: 10HZ
        time.sleep(0.1)
        device.writeReg(0x23, 0)  # Installation direction: horizontal, vertical
        time.sleep(0.1)
        device.writeReg(0x24, 0)  # Installation direction: nine-axis, six-axis
        time.sleep(0.1)
        device.save()

    def AccelerationCalibration(self, device):
        """
        Acceleration calibration
        """
        device.AccelerationCalibration()
        print("加计校准结束")

    def FiledCalibration(self, device):
        """
        Magnetic field calibration
        """
        device.BeginFiledCalibration()
        user_input = input("Please rotate slowly around the XYZ axis once each. When the three axes have completed the rotation, end the calibration (Y/N)?").lower()
        if user_input == "y":
            device.EndFiledCalibration()
            print("End magnetic field calibration")

    def onUpdate(self, deviceModel):
        """
         (Data update event)
        """
        # Print data for debugging
        # print(
        #     "芯片时间:", deviceModel.getDeviceData("Chiptime"),
        #     "温度:", deviceModel.getDeviceData("temperature"),
        #     "加速度:", deviceModel.getDeviceData("accX"), deviceModel.getDeviceData("accY"), deviceModel.getDeviceData("accZ"),
        #     "角速度:", deviceModel.getDeviceData("gyroX"), deviceModel.getDeviceData("gyroY"), deviceModel.getDeviceData("gyroZ"),
        #     "角度:", deviceModel.getDeviceData("angleX"), deviceModel.getDeviceData("angleY"), deviceModel.getDeviceData("angleZ"),
        #     "磁场:", deviceModel.getDeviceData("magX"), deviceModel.getDeviceData("magY"), deviceModel.getDeviceData("magZ"),
        #     "经度:", deviceModel.getDeviceData("lon"),
        #     "纬度:", deviceModel.getDeviceData("lat"),
        #     "航向角:", deviceModel.getDeviceData("Yaw"),
        #     "地速:", deviceModel.getDeviceData("Speed"),
        #     "四元素:", deviceModel.getDeviceData("q1"), deviceModel.getDeviceData("q2"),
        #               deviceModel.getDeviceData("q3"), deviceModel.getDeviceData("q4"),
        # )

        # 1) Publish MagneticField
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = "witmotion_link"  # or any frame_id for your setup
        
        # NOTE: If your sensor data is in microtesla (µT), convert to tesla (T) by dividing by 1e6
        # For now, we publish the raw values as floats
        mx = float(deviceModel.getDeviceData("magX") or 0.0)/100 - 3.78
        my = float(deviceModel.getDeviceData("magY") or 0.0)/100 + 4.096
        mz = float(deviceModel.getDeviceData("magZ") or 0.0)/100 + 2.163

        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz

        # You can set the covariance if known; here we set it to zero
        mag_msg.magnetic_field_covariance = [0.0]*9

        self.mag_publisher.publish(mag_msg)

        heading_radians = math.atan2(my, mx)            # Range: -π to +π
        heading_degrees = heading_radians * 180.0 / math.pi
        if heading_degrees < 0:
            heading_degrees += 360.0  # Convert range to [0, 360)
            
        # Publish heading
        heading_msg = Float32()
        heading_msg.data = heading_degrees
        self.angle_publisher.publish(heading_msg)

        # 2) If we're recording data to file, write to _writeF
        if _IsWriteF:
            Tempstr = (
                f" {deviceModel.getDeviceData('Chiptime')}\t"
                f"{deviceModel.getDeviceData('accX')}\t{deviceModel.getDeviceData('accY')}\t{deviceModel.getDeviceData('accZ')}\t"
                f"{deviceModel.getDeviceData('gyroX')}\t{deviceModel.getDeviceData('gyroY')}\t{deviceModel.getDeviceData('gyroZ')}\t"
                f"{deviceModel.getDeviceData('angleX')}\t{deviceModel.getDeviceData('angleY')}\t{deviceModel.getDeviceData('angleZ')}\t"
                f"{deviceModel.getDeviceData('temperature')}\t"
                f"{deviceModel.getDeviceData('magX')}\t{deviceModel.getDeviceData('magY')}\t{deviceModel.getDeviceData('magZ')}\t"
                f"{deviceModel.getDeviceData('lon')}\t{deviceModel.getDeviceData('lat')}\t"
                f"{deviceModel.getDeviceData('Yaw')}\t{deviceModel.getDeviceData('Speed')}\t"
                f"{deviceModel.getDeviceData('q1')}\t{deviceModel.getDeviceData('q2')}\t"
                f"{deviceModel.getDeviceData('q3')}\t{deviceModel.getDeviceData('q4')}\r\n"
            )
            _writeF.write(Tempstr)

    def startRecord(self):
        """
        开始记录数据 (Start recording data)
        """
        global _writeF, _IsWriteF
        filename = datetime.datetime.now().strftime('%Y%m%d%H%M%S') + ".txt"
        _writeF = open(filename, "w")
        _IsWriteF = True

        header = (
            "Chiptime\tax(g)\tay(g)\taz(g)\t"
            "wx(deg/s)\twy(deg/s)\twz(deg/s)\t"
            "AngleX(deg)\tAngleY(deg)\tAngleZ(deg)\t"
            "T(°)\tmagx\tmagy\tmagz\t"
            "lon\tlat\tYaw\tSpeed\t"
            "q1\tq2\tq3\tq4\r\n"
        )
        _writeF.write(header)
        print(f"开始记录数据: {filename}")

    def endRecord(self):
        """
        结束记录数据 (End record data)
        """
        global _writeF, _IsWriteF
        _IsWriteF = False
        if _writeF:
            _writeF.close()
            _writeF = None
        print("结束记录数据")
        
    def updateData(self):
        """
        Called by the self.timer to register onUpdate with the device's data processor.
        """
        self.device.dataProcessor.onVarChanged.append(self.onUpdate)

def main(args=None):
    rclpy.init(args=args)
    node = WitMotionMagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
