import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from .lidar_utils import closeOpenNozzle


class Nozzle(Node):
    def __init__(self):
        super().__init__("OnOff_nozzle_node")
        self.nozzle = closeOpenNozzle.closeOpen()
        self.subscription = self.create_subscription(
            Bool, "/nozzle_close_open", self.callbackBool, 1)

    def callbackBool(self, msg: Bool):
        if self.nozzle.serial_is_open:
            if (msg.data):
                self.nozzle.open()
            else:
                self.nozzle.close()
        pass


def main():
    rclpy.init()
    node = Nozzle()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
