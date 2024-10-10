# serial_handler.py
import serial
import threading
import time

class SerialHandler:
    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.alive = False
        self.read_callback = None
        self.lock = threading.Lock()

    def open(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.alive = True
            threading.Thread(target=self.read_loop, daemon=True).start()
            print(f"Opened serial port {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")
            self.alive = False

    def close(self):
        self.alive = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"Closed serial port {self.port}.")

    def read_loop(self):
        while self.alive and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').rstrip()
                    if self.read_callback:
                        self.read_callback(line)
                else:
                    time.sleep(0.01)  # Prevent CPU hogging
            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                self.alive = False
            except UnicodeDecodeError as e:
                print(f"Decode error: {e}")

    def send(self, data):
        with self.lock:
            if self.serial and self.serial.is_open:
                try:
                    self.serial.write((data + '\n').encode('utf-8'))
                    print(f"Sent: {data}")
                except serial.SerialException as e:
                    print(f"Serial write error: {e}")
