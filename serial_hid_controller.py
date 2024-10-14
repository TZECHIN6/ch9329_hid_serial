import serial
from dataclasses import dataclass, field
from typing import List
from enum import Enum
import time


PORT = "/dev/ttyUSB0"
BAUDRATE = 38400


class HIDProperty(Enum):
    HID_KEY = 0x01
    HID_MKEY = 0x02
    HID_MOUSE = 0x05


@dataclass
class Frame:
    HEADER: int = field(default=0x2C, init=False)
    data_length: int = field(init=False)
    property: str
    crc: int = field(init=False)
    hid_data: List[int]

    def __post_init__(self):
        self.property = self.define_property(self.property)
        self.data_length = self.calculate_data_length()
        self.crc = self.calculate_crc()

    def calculate_crc(self) -> int:
        crc_sum = sum(self.hid_data) + self.property
        return crc_sum & 0xFF

    def calculate_data_length(self) -> int:
        return len(self.hid_data)

    def define_property(self, property_name: str) -> int:
        try:
            return HIDProperty[property_name].value
        except KeyError:
            raise ValueError(f"Invalid property name: {property_name}")

    def to_bytes(self) -> bytes:
        frame = [self.HEADER, self.data_length, self.crc, self.property] + self.hid_data
        return bytes(frame)


class SerialHIDController:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect()

    def connect(self):
        """Establish the serial connection."""
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate)
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.port}: {e}")
            self.ser = None

    def disconnect(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_frame(self, frame: Frame) -> bool:
        """
        1. Check the connection is ready or not.
        2. Convert the data frame to bytes.
        3. Send the serial data.
        4. Check the sent serial data is complete or not.

        Args:
            frame (Frame): A complete data frame object
            constructed by the dataclass Frame.

        Returns:
            bool: Check if the serial data is sent successfully or not.
        """
        if not self.ser or not self.ser.is_open:
            print("Serial port is not oepn. Cannot send data frame.")
            return False
        frame_byte = frame.to_bytes()
        try:
            bytes_written = self.ser.write(frame_byte)
            if bytes_written != len(frame_byte):
                print("Failed to write complete frame to serial port.")
                return False
            return True
        except serial.SerialException as e:
            print(f"Failed to write to serial port: {e}")
            return False

    def volume_up(self):
        """First send a serial frame to press volume up key,
        then send a serial frame to release all key.
        """
        frame = Frame(
            property="HID_MKEY", hid_data=[0xE9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)
        frame = Frame(
            property="HID_MKEY", hid_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)

    def volume_down(self):
        """First send a serial frame to press volume down key,
        then send a serial frame to release all key.
        """
        frame = Frame(
            property="HID_MKEY", hid_data=[0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)
        frame = Frame(
            property="HID_MKEY", hid_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)


serController = SerialHIDController(PORT, BAUDRATE)
for i in range(10):
    serController.volume_up()
    time.sleep(1 / 30)
for i in range(10):
    serController.volume_down()
    time.sleep(1 / 30)
serController.disconnect()
