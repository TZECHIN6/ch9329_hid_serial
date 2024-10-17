# CH9329 HID Serial
A Python package to effortlessly integrate CH9329 HID Serial control into your Python projects with intuitive and simple commands.

## Installation
```
pip3 install -r requirements.txt
```

## Usage
1. Connect the bigger USB socket to host PC (send command), and connect the smaller USB socket to client PC (receive command).
2. Make sure the com port is available. (might require sudo premission)

```
# Linux (example)
$ sudo chmod 666 /dev/ttyUSB0
```

3. Update the config parameters so that it fits to your system.
4. Import the module and use it :wink:.
```
from ch9329_hid_serial.serial_hid_controller import SerialHIDController

ser = SerialHIDController()  # Initial the serial object

# Example actions
ser.volume_up()
ser.volume_down()
ser.next_slide()
ser.previous_slide()
ser.move_relative(dx=50, dy=-50)
ser.move_absolute(x=500, y=500)
ser.mouse_click()

ser.disconnect()  # Close the serial communication
```

## Design Concept
1. This package is trying to intergrate the basic command or basic human input action with simple function calls. Therefore, for example, go to next slide action is expected the target PC already in presentation mode.
2. Another important point is that for complex action (mostly involves serval different simple action) such as drag (first move the mouse to target x and y postion, next perform pressing mouse button, then keep track of user relative movement and move the mouse relatively, finally release the pressed mouse botton) using a state-machine like logic to decide which basic command should be sent is preferred.

