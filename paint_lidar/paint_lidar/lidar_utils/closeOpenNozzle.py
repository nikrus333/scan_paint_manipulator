import serial
import time

class closeOpen:
    def __init__(self, port = '/dev/ttyAMC0', baudrate = 57600) -> None:
        try:
            self.port = port
            self.baudrate = baudrate
            self.nozzle_open = False
            self.SerialObj = serial.Serial(port=self.port, baudrate=self.baudrate)
            print("Connecting...")
            while not self.SerialObj.readline():
                continue
            print("Device is ready!")
            time.sleep(0.2)
            self.serial_is_open = True

        except serial.SerialException:
            print(f"Not found {self.port}")
            self.serial_is_open = False
            pass

    def get_connect(self):
        return self.serial_is_open()

    def check_connect(self):
        if self.SerialObj.readline():
            self.serial_is_open = True
            print("ok")
            return True
        else:
            self.serial_is_open = False
            self.reconnect()
            return False
        
    def reconnect(self):
        self.closePort()
        self.SerialObj = serial.Serial(port=self.port, baudrate=self.baudrate)
        self.check_connect()
        print("Reconnect to device")

    def close(self):
        while self.nozzle_open != False:
            data_str = "Off"
            time.sleep(0.1)
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            mess = self.SerialObj.readline()
            if mess == bytes(b'Close\r\n'):
                self.nozzle_open = False
            else:
                print("Error message sending!")

    def open(self):
        while self.nozzle_open != True:
            data_str = "On"
            time.sleep(0.1)
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            mess = self.SerialObj.readline()
            if mess == bytes(b'Open\r\n'):
                self.nozzle_open = True
            else:
                print("Error message sending!")

    def closePort(self):
        self.SerialObj.close()

# class closeOpen:
#     def __init__(self, port = '/dev/ttyACM2', baudrate = 57600) -> None:
#         try:
#             self.port = port
#             self.baudrate = baudrate
#             self.nozzle_open = False
#             self.SerialObj = serial.Serial(port=self.port, baudrate=self.baudrate)
#             print("Loading...")
#             #self.SerialObj.readline()
#             print("Ready!")
#             time.sleep(0.2)
#             self.serial_is_open = True

#         except serial.SerialException:
#             print(f"Not found {self.port}")
#             self.serial_is_open = False
#             pass

#     def close(self):
#         if self.nozzle_open != False:
#             data_str = "Off"
#             out = data_str.encode('utf-8')
#             self.SerialObj.write(out)
#             self.nozzle_open = False
#             print("Close")

#     def open(self):
#         if self.nozzle_open != True:
#             data_str = "On"
#             out = data_str.encode('utf-8')
#             self.SerialObj.write(out)
#             self.nozzle_open = True
#             print("Open")

#     def closePort(self):
#         self.SerialObj.close()