#!/usr/bin/python3

import serial
import time
import sys

if len(sys.argv) < 2:
    print("Usage: liveuart_draw.py [-c config] <port>")
    sys.exit(1)

if sys.argv[1] == "-c":
    config = sys.argv[2]
    port = sys.argv[3]
else:
    config = None
    port = sys.argv[1]

ser = serial.Serial(port, 2000000)

def tx(cmd, data):
    ser.write(b'\xAA' + (len(data)+1).to_bytes(2, 'big') + bytes([cmd]) + data)

def overlay_state(state):
    tx(8, bytes([state]))

def uart_cursor(x, y):
    tx(4, bytes([x, y]))

def uart_print(s):
    b = s.encode('utf-8')
    tx(5, b)

overlay_state(1)

# clear the 32x28 screen
for y in range(28):
    for x in range(32):
        uart_cursor(0, y)
        #           01234567890123456789012345678901
        uart_print("                                ")

uart_cursor(0, 0)
uart_print("Hello, world!")

uart_cursor(0, 1)
uart_print(">")
tx(1, bytes())           # get core ID
time.sleep(0.5)
uart_cursor(0, 2)
uart_print("core_id")

# draw a moving ball
# x = 0
# y = 1
# while True:
#     uart_cursor(x, y)
#     uart_print("O")
#     time.sleep(0.1)
#     uart_cursor(x, y)
#     uart_print(" ")
#     x += 1
#     if x >= 32:
#         x = 0
#         y += 1
#     if y >= 28:
#         y = 1

