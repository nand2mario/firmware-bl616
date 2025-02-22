#!/usr/bin/python3

import serial
import time
import sys

if len(sys.argv) != 2:
    print("Usage: liveuart.py <port>")
    sys.exit(1)

port = sys.argv[1]

ser = serial.Serial(port, 1000000)

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

def handle_load_data():
    global newline
    # Read length (3 bytes)
    length = ser.read(3)
    length = int.from_bytes(length, 'big')
    # Read data
    data = ser.read(length)
    print(f"<load_data:{length}> {data[:8].hex()}")

def handle_overlay_state():
    global newline
    # Read state (1 byte)
    state = ser.read(1)
    print(f"<overlay_state:{state}>")

while True:
    command = ser.read(1)
    if not command:
        continue
        
    if command == b'\x04':  # Command 4 - Cursor Move
        handle_cursor_move()
    elif command == b'\x05':  # Command 5 - Print
        handle_print()
    elif command == b'\x01':  # Command 1 - get core id
        print("<get_core_id>")
    elif command == b'\x06':  # Command 6 - set loading state
        st = ser.read(1)
        print(f"<set_loading_state:{st}>")
    elif command == b'\x07':  # Command 7 - load data
        handle_load_data()
    elif command == b'\x08':  # Command 8 - set overlay state
        handle_overlay_state()
    else:
        print(f"Unknown command: {command}")
    


