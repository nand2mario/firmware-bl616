#!/usr/bin/python3

import serial
import time
import sys

if len(sys.argv) != 2:
    print("Usage: liveuart.py <port>")
    sys.exit(1)

port = sys.argv[1]

ser = serial.Serial(port, 2000000)

newline = False

def handle_cursor_move():
    global newline
    # Read x and y coordinates (2 bytes) but ignore them
    ser.read(2)
    newline = True

def handle_print():
    global newline
    # Read until null terminator
    string = b''
    while True:
        char = ser.read(1)
        if char == b'\x00':
            break
        string += char
    s = string.decode('utf-8')
    # hack to remove menu '>' from the output
    if newline and (s=='>' or s==' '):
        return
    if newline:
        print()
        newline = False
    print(string.decode('utf-8'), end='')

while True:
    command = ser.read(1)
    if not command:
        continue
        
    if command == b'\x04':  # Command 4 - Cursor Move
        handle_cursor_move()
    elif command == b'\x05':  # Command 5 - Print
        handle_print()
    else:
        print(f"Unknown command: {command}")
    


