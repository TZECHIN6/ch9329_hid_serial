import os
import serial
import yaml
from dataclasses import dataclass, field
from typing import List
from enum import Enum

import time


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
    def __init__(self):
        self.ser = None
        self.load_config()
        self.connect()
        self.display_info()

    def load_config(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(
            current_dir, "config", "serial_hid_controller_param.yaml"
        )
        with open(config_path, "r") as file:
            config = yaml.safe_load(file)
        self.port: str = config["serial"]["port"]
        self.baudrate: int = config["serial"]["baudrate"]
        self.timeout: float = config["serial"]["timeout"]
        self.screen_x_max: int = config["target_screen_resolution"]["x_max"]
        self.screen_y_max: int = config["target_screen_resolution"]["y_max"]

    def display_info(self):
        info = (
            "---------------------------------------------\n"
            "Successfully Established Serial Communication\n\n"
            "Port:                     {self.port}\n"
            "Baudrate:                 {self.baudrate}\n"
            "Timeout:                  {self.timeout}\n"
            "Target Screen Resolution: {self.screen_x_max} x {self.screen_y_max}\n"
            "---------------------------------------------"
        )
        print(info.format(self=self))

    def connect(self):
        """Establish the serial connection."""
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=self.timeout
            )
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
        self.hid_release("HID_MKEY")

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
        self.hid_release("HID_MKEY")

    def next_slide(self):
        """
        First send a serial frame to press right arrow key,
        then send a serial frame to release all key.
        """
        frame = Frame(
            property="HID_KEY",
            hid_data=[0x00, 0x00, 0x4F, 0x00, 0x00, 0x00, 0x00, 0x00],
        )
        self.send_frame(frame)
        self.hid_release("HID_KEY")

    def previous_slide(self):
        """
        First send a serial frame to press left arrow key,
        then send a serial frame to release all key.
        """
        frame = Frame(
            property="HID_KEY",
            hid_data=[0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00],
        )
        self.send_frame(frame)
        self.hid_release("HID_KEY")

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
        if (dx > 127 or dx < -128) or (dy > 127 or dy < -128):
            print(
                "The required relative movement is out of range. No HID command will be sent."
            )
            return
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
        x = (4096 * x) // self.screen_x_max
        y = (4096 * y) // self.screen_y_max
        x = int(x)
        y = int(y)
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

    def mouse_click(self):
        """
        First send a serial frame to press mouse left button,
        then send a serial frame to release all mouse button.
        """
        self.mouse_press()
        self.hid_release("HID_MOUSE")

    def hid_release(self, hid_type: str):
        """To release all the pressed key/button of the given HID device.

        Args:
            hid_type (str): The type of the desired HID device to perform release.
            Only below listed arguments are accepted.
            1. HID_KEY
            2. HID_MKEY
            3. HID_MOUSE
        """
        if hid_type == "HID_KEY":
            frame = Frame(
                property="HID_KEY",
                hid_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            )
            self.send_frame(frame)
        elif hid_type == "HID_MKEY":
            frame = Frame(
                property="HID_MKEY", hid_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            )
            self.send_frame(frame)
        elif hid_type == "HID_MOUSE":
            frame = Frame(
                property="HID_MOUSE",
                hid_data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            )
            self.send_frame(frame)
        else:
            print("Invalid HID type. No release action is sent.")


def main():
    serController = SerialHIDController()
    for _ in range(5):
        serController.volume_down()
        time.sleep(0.5)
    for _ in range(5):
        serController.volume_up()
        time.sleep(0.5)
    serController.disconnect()


if __name__ == "__main__":
    main()
