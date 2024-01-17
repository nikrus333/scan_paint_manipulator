from scipy.spatial.transform import Rotation as R
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from example_interfaces.action import ExecuteTrajectoryArray
from example_interfaces.srv import SetBool, TrajectoryMode

from .lidar_utils.enum_set import ModeWork, ChooseStartManip


class MinimalClientAsync(Node):

    def __init__(self):
        
        super().__init__('menu_client')
        self.client = self.create_client(TrajectoryMode, '/service_trajectory')
        attempted = 0
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service trajectory not available')
            attempted += 1
            if attempted >= 5:
                print("Couldn`t to connect to service_trajectory")
                break

        self.points = None
        self.angle = None
        self.rotation = None
        self.trans = None
        self.count_paint_make = 0

    def send_request_trajectory(self, req: TrajectoryMode.Request) -> TrajectoryMode.Response:
        self.future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def chose_mode_work(self) -> int:
        while True:
            print('Введите номер режима работы')
            print(f'{ModeWork.SCAN_AND_PAINT.value} - автоматическое сканирование и обработка')
            print(f'{ModeWork.CALCULATE_RANGE.value} - работа с g-code')
            mode_work = input()
            if mode_work.isdigit():
                mode_work = int(mode_work)
                number_operation = ModeWork.G_CODE_MODE.value
            else:
                print('Введите номер операции')
                print(f'{ModeWork.SCAN_AND_PAINT.value} - сканировние и окраска')
                print(f'{ModeWork.CALCULATE_RANGE.value} - расстояние до поверхности')
                print(f'{ModeWork.ONLY_PAINT.value} - окраска, без сканирования')
                number_operation = input()
                if number_operation.isdigit():
                    number_operation = int(number_operation)
                if (number_operation == ModeWork.SCAN_AND_PAINT.value) or (number_operation == ModeWork.ONLY_PAINT.value) or (number_operation == ModeWork.CALCULATE_RANGE.value):
                    break
                else:
                    print('Некорректный ввод')
            return number_operation

    def chose_start_manip(self) -> int:
        while True:
            print('Проверьте точки траектории')
            print('Точки траектории корректны')
            print(f'{ChooseStartManip.START_MANIPULATOR.value} - Да')
            print(f'{ChooseStartManip.SKIP_TRAJECTORY.value} - Нет')
            chose_start_manip = input()
            if chose_start_manip.isdigit():
                chose_start_manip = int(chose_start_manip)
            if (chose_start_manip == ChooseStartManip.SKIP_TRAJECTORY.value) or (chose_start_manip == ChooseStartManip.START_MANIPULATOR.value):
                return chose_start_manip
            else:
                print('Некорректный ввод')


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.count_paint_make = 0
    request = TrajectoryMode.Request()
    while True:
        print('______________________________________')
        mode_work = minimal_client.chose_mode_work()
        match mode_work:
            case ModeWork.SCAN_AND_PAINT.value:
                print("Wait until the scan is completed 🐢")
                request.mode_work = mode_work
                request.choose_start_manip = 0
                response = minimal_client.send_request_trajectory(request)
                if response.success:
                    print(response.message)

            case ModeWork.CALCULATE_RANGE.value:
                request.mode_work = mode_work
                request.choose_start_manip = 0
                response = minimal_client.send_request_trajectory(request)
                if response.success and response.message != "":
                    print(response.message)

            case ModeWork.ONLY_PAINT.value:
                print("Wait until the paint is completed 🐢")
                request.mode_work = mode_work
                request.choose_start_manip = 0
                response = minimal_client.send_request_trajectory(request)
                if response.success:
                    choose_start_manip = minimal_client.chose_start_manip()
                    match choose_start_manip:
                        case ChooseStartManip.START_MANIPULATOR.value:
                            request.mode_work = mode_work
                            request.choose_start_manip = choose_start_manip
                            response = minimal_client.send_request_trajectory(request)
                        case ChooseStartManip.SKIP_TRAJECTORY.value:
                            request.mode_work = mode_work
                            request.choose_start_manip = choose_start_manip
                            response = minimal_client.send_request_trajectory(request)
                            pass 
            case ModeWork.G_CODE_MODE.value:
                print("Wait until the G-code programm is completed 🐢")
                request.mode_work = mode_work
                request.choose_start_manip = 0
                response = minimal_client.send_request_trajectory(request)
        print('______________________________________')
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
