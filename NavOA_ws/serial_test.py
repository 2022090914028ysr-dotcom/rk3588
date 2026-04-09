#!/usr/bin/env python3
import serial
import time

# Open serial port
ser = serial.Serial('/dev/ttyS7', 230400, timeout=1)

# Send text
ser.write(b"Hello from Python!\n")

# Send hex bytes
ser.write(b'\x01\x02\x03\x04')

# Send with encoding
ser.write("中文消息\n".encode('utf-8'))

# Send multiple messages
for i in range(10):
    ser.write(f"Message {i} 你好，世界！\n".encode())
    time.sleep(1)  # Wait a bit between messages

ser.close()
print("Messages sent!") 