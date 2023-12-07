import serial
import time

class closeOpen:
    def __init__(self, simulation=False) -> None:
        self.simulation = simulation
        if not simulation:
            self.SerialObj = serial.Serial(port="/dev/ttyServo", baudrate=115200)
            # self.SerialObj = serial.Serial(port="/dev/ttyACM2", baudrate=115200)
            print("Waiting servo...")
            self.ckeckState()
            self.servo_open = False
            time.sleep(1)
            print('Servo is ready!')
        else:
            print('simulation mode for servo lidar')

    def ckeckState(self):
        while True:
            data_str = "check"
            time.sleep(0.5)
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            mess = self.SerialObj.readline()
            if mess == bytes(b'Ready\r\n'):
                return True

    def close(self):
        while self.servo_open != False:
            data_str = "Off"
            time.sleep(0.1)
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            mess = self.SerialObj.readline()
            if mess == bytes(b'Close\r\n'):
                self.servo_open = False
                # print("F")
            else:
                print("Error message sending!")

    def open(self):
        while self.servo_open != True:
            data_str = "On"
            time.sleep(0.1)
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            mess = self.SerialObj.readline()
            if mess == bytes(b'Open\r\n'):
                self.servo_open = True
                # print("T")
            else:
                print("Error message sending!")

    def closePort(self):
        self.SerialObj.close()

    def listener(self):
        out = self.SerialObj.readline()
        print(out)


if __name__ == '__main__':
    a = closeOpen()
    while True:
        # a.listener()
        time.sleep(1)
        print('open')
        a.open()
        # a.listener()
        time.sleep(2)
        print('close')
        a.close()
        # a.listener()
        time.sleep(2)