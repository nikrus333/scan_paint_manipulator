import rclpy
from rclpy.node import Node
import time, serial

from example_interfaces.srv import OpenClose
from paint_lidar.lidar_utils.closeOpenNozzle import closeOpen
from .lidar_utils.enum_set import DevParametrs, SelectModeWork
class MinimalService(Node):
    def __init__(self):
        super().__init__('server_open_close')
        self.srv = self.create_service(OpenClose, 'service_nozzle', self.status_callback)
        if not SelectModeWork.NOZZLE_SIMULATION.value:
            self.serial = closeOpen(port=DevParametrs.NOZZLE_DEV.value, baudrate=115200)
            self.get_logger().info(f"Connection: {self.serial.get_connect()}")

    def status_callback(self, request: OpenClose.Request, response: OpenClose.Response) -> OpenClose.Response:
        if (request.status):
            if not SelectModeWork.NOZZLE_SIMULATION.value:
                if self.serial.serial_is_open:
                    self.serial.open(debug=True)
                else:
                    self.get_logger().info(f"Serial not opened")
                    response = "Serial close"
                    return response
            response.result = "Nozzle is open"
        else:
            if not SelectModeWork.NOZZLE_SIMULATION.value:
                if self.serial.serial_is_open:
                    self.serial.close()
                else:
                    self.get_logger().info(f"Serial not opened")
                    response = "Serial close"
                    return response
            response.result = "Nozzle is close"
        self.get_logger().info(f'Incoming request: {request.status}')  # CHANGE
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()