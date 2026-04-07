#!/usr/bin/env python3
"""
UART Loopback Test for Jetson (/dev/ttyTHS1)

Steps:
1. Connect TX and RX pins together.
2. Run this script.
3. It sends a test message and checks if it was received correctly.
"""

import serial
import time

# --- Configuration ---
PORT = "/dev/ttyTHS1"
BAUDRATE = 115200
TEST_MESSAGE = b"Loopback test message\n"

# --- Initialize UART ---
ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    timeout=1
)

if not ser.is_open:
    ser.open()

print(f"Opened {PORT} at {BAUDRATE} baud")

# --- Flush buffers ---
ser.reset_input_buffer()
ser.reset_output_buffer()

# --- Send test data ---
print(f"Sending: {TEST_MESSAGE.decode().strip()}")
ser.write(TEST_MESSAGE)
time.sleep(0.1)  # short delay to allow loopback

# --- Read response ---
received = ser.read(len(TEST_MESSAGE))

# --- Check result ---
if received == TEST_MESSAGE:
    print("✅ Loopback successful!")
else:
    print("❌ Loopback failed!")
    print(f"Received: {received}")

# --- Cleanup ---
ser.close()
print("Port closed.")
