import rclpy
from rclpy.node import Node
import time, serial

from example_interfaces.srv import OpenClose
from paint_lidar.lidar_utils.closeOpenNozzle import closeOpen

class MinimalService(Node):
    def __init__(self):
        super().__init__('server_open_close')
        self.srv = self.create_service(OpenClose, 'service_open', self.status_callback)
        self.serial = closeOpen(port='/dev/ttyNozzle', baudrate=115200)
        # self.serial = closeOpen(port='/dev/ttyACM1', baudrate=115200)

    def status_callback(self, request, response):
        if self.serial.serial_is_open:
            if (request.status):
                self.serial.open()
                response.result = "Nozzle is open"
            else:
                self.serial.close()
                response.result = "Nozzle is close"
            self.get_logger().info(f'Incoming request: {request.status}')  # CHANGE
            return response
        else:
            self.get_logger().info(f"Serial not opened")
            response = "Serial close"
            return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()