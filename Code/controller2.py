import serial
import struct
import time

# Set your Arduino's COM port
arduino_port = "COM3" # Example for Windows
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
    
time.sleep(2)  # Wait for the Arduino to initialize

# Send a 16-bit unsigned integer
send_uint16(12345)
while True:
    received_value = read_uint16()
    if received_value is not None:
        print("Received value:", received_value)
        break
    else:
        print("Error: No data received")
    time.sleep(0.5)  # Add a delay to avoid overloading the console output
    
ser.close()  # Close the serial connection