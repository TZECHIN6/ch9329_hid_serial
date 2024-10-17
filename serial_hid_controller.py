import serial
from dataclasses import dataclass, field
from typing import List
from enum import Enum
import time


PORT = "/dev/ttyUSB0"  # CH9329 USB device
BAUDRATE = 38400
SCREEN_X_MAX = 1920  # target screen x-resolution
SCREEN_Y_MAX = 1200  # target screen y-resolutoin


class HIDProperty(Enum):
    HID_KEY = 0x01
    HID_MKEY = 0x02
    HID_MOUSE = 0x05
    HID_ABSMOUSE = 0x06


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

    def frame_to_bytes(self) -> bytes:
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
            exit()

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
        frame_byte = frame.frame_to_bytes()
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
        """
        First send a serial frame to press volume up key,
        then send a serial frame to release all key.

        [INFO] Each execution increase the volume by 2.
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
        """
        First send a serial frame to press volume down key,
        then send a serial frame to release all key.

        [INFO] Each execution decrease the volume by 2.
        """
        frame = Frame(
            property="HID_MKEY", hid_data=[0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)
        frame = Frame(
            property="HID_MKEY", hid_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)

    def move_relative(self, dx: int, dy: int):
        """
        Move the mouse to a given relative position.

        [INFO] A positive value (0 to 127) represents forward/right movement.
        A negative value (-1 to -128) represents backward/left movement,
        but in the byte form it goes from 255 down to 128.

        [WARN] Relative movement in one direction greater than 127 would result
        in your movement being incorrectly interpreted.

        Args:
            dx (int): Relative x position
            dy (int): Relative y position
        """
        if dx < 0:
            dx = 256 + dx
        if dy < 0:
            dy = 256 + dy
        frame = Frame(
            property="HID_MOUSE",
            hid_data=[0x00, 0x00, dx, dy, 0x00, 0x00, 0x00],
        )
        self.send_frame(frame)

    def move_absolute(self, x: int, y: int):
        """
        Move the mouse to a given absolute position.

        [INFO] 4096: The maximum value for a 12-bit integer (0-4095),
        which is typically used in HID absolute positioning.

        [WARN] This function might only work in Windows OS.

        Args:
            x (int): Desired absolute x position
            y (int): Desired absolute y position
        """
        x = (4096 * x) // SCREEN_X_MAX
        y = (4096 * y) // SCREEN_Y_MAX
        x_bytes: bytes = x.to_bytes(2, "little")
        y_bytes: bytes = y.to_bytes(2, "little")
        frame = Frame(
            property="HID_ABSMOUSE",
            hid_data=[0x00, 0x00, x_bytes[0], x_bytes[1], y_bytes[0], y_bytes[1], 0x00],
        )
        self.send_frame(frame)

    def mouse_press(self):
        """Perform pressing left mouse button."""
        frame = Frame(
            property="HID_MOUSE", hid_data=[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)

    def mouse_release(self):
        frame = Frame(
            property="HID_MOUSE", hid_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_frame(frame)


# Example
serController = SerialHIDController(PORT, BAUDRATE)
for i in range(5):
    serController.volume_down()
    time.sleep(0.5)
for i in range(5):
    serController.volume_up()
    time.sleep(0.5)
serController.disconnect()
