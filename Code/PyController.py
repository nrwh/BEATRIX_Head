import keyboard
import serial
import struct
import time

# Set your Arduino's COM port
arduino_port = "COM7" # Example for Windows
# arduino_port = "/dev/ttyACM0" # Example for Linux

# Open serial connection
ser = serial.Serial(arduino_port, 9600)

def send_uint16(value):
    if 0 <= value <= 65535:
        packed_data = struct.pack("<H", value)
        ser.write(packed_data)
    else:
        raise ValueError("Value must be between 0 and 65535.")

def read_uint16():
    data = ser.read(2)
    if len(data) == 2:
        value = struct.unpack("<H", data)[0]
        return value
    else:
        return None

time.sleep(2)  # Wait for the Arduino to initial

while not keyboard.is_pressed("z"):
    time.sleep(0.1)
    stepperX_forward = keyboard.is_pressed("q")
    stepperX_backward = keyboard.is_pressed("a")
    stepperY_forward = keyboard.is_pressed("w")
    stepperY_backward = keyboard.is_pressed("s")
    stepperZ_forward = keyboard.is_pressed("e")
    stepperZ_backward = keyboard.is_pressed("d")

    servo0_forward = keyboard.is_pressed("r")
    servo0_backward = keyboard.is_pressed("f")
    servo1_forward = keyboard.is_pressed("t")
    servo1_backward = keyboard.is_pressed("g")
    servo2_forward = keyboard.is_pressed("y")
    servo2_backward = keyboard.is_pressed("h")
    servo3_forward = keyboard.is_pressed("u")
    servo3_backward = keyboard.is_pressed("j")

    StepperX = ((2 * stepperX_forward) + stepperX_backward) * (not (stepperX_forward and stepperX_backward))
    StepperY = ((2 * stepperY_forward) + stepperY_backward) * (not (stepperY_forward and stepperY_backward))
    StepperZ = ((2 * stepperZ_forward) + stepperZ_backward) * (not (stepperZ_forward and stepperZ_backward))

    servo0 = (2 * servo0_forward + servo0_backward) * (not (servo0_forward and servo0_backward))
    servo1 = (2 * servo1_forward + servo1_backward) * (not (servo1_forward and servo1_backward))
    servo2 = (2 * servo2_forward + servo2_backward) * (not (servo2_forward and servo2_backward))
    servo3 = (2 * servo3_forward + servo3_backward) * (not (servo3_forward and servo3_backward)) 

    sendValue = (StepperX * (2**0)) + (StepperY * (2**2)) + (StepperZ * (2**4)) + (servo0 * (2**6)) + (servo1 * (2**8)) + (servo2 * (2**10)) + (servo3 * (2**12)) + (1 * 2**15)
    send_uint16(sendValue)
    print(sendValue)
    # while True:
    #     received_value = read_uint16()
    #     if received_value is not None:
    #         print("Received value:", received_value)
    #         break
    #     else:
    #         print("Error: No data received")
    #     time.sleep(0.5)  # Add a delay to avoid overloading the console output 