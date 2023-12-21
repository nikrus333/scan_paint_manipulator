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

    def get_connect(self) -> bool:
        return self.serial_is_open

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

    def close(self, debug=False):
        while self.nozzle_open != False:
            data_str = "f"
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            mess = self.SerialObj.readline()
            if debug: 
                print(mess)
            if mess == bytes(b'Close\r\n'):
                self.nozzle_open = False
            else:
                print("Error message sending!")
                break

    def open(self, debug=False):
        while self.nozzle_open != True:
            data_str = "n"
            out = data_str.encode('utf-8')
            self.SerialObj.write(out)
            mess = self.SerialObj.readline()
            if debug: 
                print(mess)
            if mess == bytes(b'Open\r\n'):
                self.nozzle_open = True
            else:
                print("Error message sending!")
                break

    def closePort(self):
        self.SerialObj.close()

if __name__ == "__main__":
    serial_nozzle = closeOpen("/dev/ttyUSB0", 115200)
    if serial_nozzle.get_connect():
        while KeyboardInterrupt:
            serial_nozzle.open(debug=True)
            time.sleep(0.5)
            serial_nozzle.close(debug=True)
            time.sleep(0.5)
    
