import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2) 

commands = [
    "PIN13:HIGH\n",
    "PIN12:LOW\n",
    "PIN11:HIGH\n"
]

# Send each command
for cmd in commands:
    arduino.write(cmd.encode())  # Convert string to bytes
    time.sleep(0.5) 

arduino.close()
