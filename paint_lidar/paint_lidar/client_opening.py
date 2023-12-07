import sys
import rclpy
from rclpy.node import Node

from example_interfaces.srv import OpenClose

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('client_open_close')
        self.cli = self.create_client(OpenClose, 'service_open')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OpenClose.Request()                                   

    def send_request(self, status: bool):
        self.req.status = status
        self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    while KeyboardInterrupt:
        status = bool(str(input("Enter status for servo: ")))
        minimal_client.send_request(status)

        while rclpy.ok():
            rclpy.spin_once(minimal_client)
            if minimal_client.future.done():
                try:
                    response = minimal_client.future.result()
                except Exception as e:
                    minimal_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    minimal_client.get_logger().info(f'Result {response.result}')  
                break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()