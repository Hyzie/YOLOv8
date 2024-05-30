import serial
import serial.tools.list_ports

class AutoCOMPort:
    def __init__(self, baudrate=9600, timeout=1, vid=None, pid=None):
        self.baudrate = baudrate
        self.timeout = timeout
        self.vid = vid
        self.pid = pid
        self.port = self.auto_select_port()
        self.serial_connection = None

        if self.port:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.port}")
        else:
            print("No suitable COM port found.")

    def auto_select_port(self):
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if self.is_correct_port(port):
                return port.device
        return None

    def is_correct_port(self, port):
        if self.vid and self.pid:
            if (port.vid == self.vid) and (port.pid == self.pid):
                return True
        return False

    def read_data(self):
        if self.serial_connection:
            return self.serial_connection.readline()
        return None

    def write_data(self, data):
        if self.serial_connection:
            self.serial_connection.write(data.encode())

    def close(self):
        if self.serial_connection:
            self.serial_connection.close()
            print(f"Disconnected from {self.port}")