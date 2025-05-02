#!/usr/bin/python3

import serial
import sys
import argparse

parser = argparse.ArgumentParser(description='Utility to view UART traffic by BL616 and FPGA with Sipeed RV-Debugger.')
parser.add_argument('port', help='COM port to connect to')
parser.add_argument('rom', nargs='?', help='Optional ROM file to load into the core')
parser.add_argument('-b', '--bl616', action='store_true', help='Decode messages from BL616 (default)')
parser.add_argument('-f', '--fpga', action='store_true', help='Decode messages from FPGA')
parser.add_argument('--unhide', action='store_true', help='Show hidden messages')

args = parser.parse_args()
port = args.port
rom = args.rom

ser = serial.Serial(port, 2000000)
newline = False

def handle_cursor_move(buf):
    global newline
    # Read x and y coordinates (2 bytes) but ignore them
    if len(buf) != 3:
        print(f"<cursor_move:BAD_COMMAND>")
        return
    if args.unhide:
        print(f"<cursor_move:{buf[1]},{buf[2]}>")
    newline = True

def handle_print(buf):
    global newline
    try:
        s = buf[1:].decode("ascii", errors="ignore")
    except UnicodeDecodeError:
        print(f"<print:len={len(buf)}, DECODE_ERROR>")
        return
    # hack to remove menu '>' from the output
    if newline and (s=='>' or s==' ' and not args.unhide):
        return
    if newline:
        print()
        newline = False
    print(s, end='')

def handle_load_data(buf):
    global newline
    # Read data
    data = buf[1:]
    print(f"<load_data:{len(data)}> {data[:8].hex()}")

def handle_overlay_state(buf):
    global newline
    # Read state (1 byte)
    print(f"<overlay_state:{buf[1]}>")

def handle_hid_to_core(buf):
    hid = buf[1:]
    print(f"<hid: {hid.hex()}>")

def handle_floppy_data(buf):
    check_len(buf, 513)
    data = buf[1:]
    print(f"<floppy_data:{data[:4].hex()}...>")

def handle_disk_mgmt(buf):
    address = int.from_bytes(buf[1:3], 'big')
    data = int.from_bytes(buf[3:5], 'big')
    print(f"<disk_mgmt:{address:04x}<={data:04x}>")

def handle_keyboard_scancode(buf):
    scancode = buf[1:]
    print(f"<keyboard_scancode:{scancode.hex()}>")

def handle_bl616_command():
    sync = ser.read(1)
    if sync != b'\xAA':
        print(f"{chr(sync[0])}", end="")
        return
    
    len = int.from_bytes(ser.read(2), 'big')
    if len >= 2048:
        print(f"Length too large: {len}")
        return

    buf = ser.read(len)
    command = buf[0]
    if not command or command == b'\x00':
        return
        
    if command == 1:  # Command 1 - get core id
        print("<get_core_id>")
    elif command == 2:  # Command 2 - get config string
        print("<get_config_string>")
    elif command == 3:  # Command 3 - get core config status
        print("<get_core_config_status>")
    elif command == 4:  # Command 4 - Cursor Move
        handle_cursor_move(buf)
    elif command == 5 or command == 13:  # Command 5 - Print
        handle_print(buf)
    elif command == 6:  # Command 6 - set loading state
        if len == 2:
            print(f"<set_loading_state:{buf[1]}>")
        else:
            print(f"<set_loading_state:BAD_COMMAND>")
    elif command == 7:  # Command 7 - load data
        print(f"<load_data:{len}>")
        # handle_load_data(buf)
    elif command == 8:  # Command 8 - set overlay state
        handle_overlay_state(buf)
    elif command == 9:  # Command 9 - send HID to core
        handle_hid_to_core(buf)
    elif command == 10:  # Command 10 - send floppy data
        handle_floppy_data(buf)
    elif command == 11:  # Command 11 - write to disk management interface
        handle_disk_mgmt(buf)
    elif command == 12:  # Command 12 - send keyboard scancode
        handle_keyboard_scancode(buf)
    else:
        print(f"Unknown command: {command}")

def check_len(buf, expected):
    if len(buf) != expected:
        print(f"<BAD_COMMAND>")
        return False
    return True

def handle_fpga_command():
    sync = ser.read(1)
    if sync != b'\xAA':
        print(f"{chr(sync[0])}", end="")
        return
    
    len = int.from_bytes(ser.read(2), 'big')
    if len >= 1024:
        print(f"Length too large: {len}")
        return

    buf = ser.read(len)    

    command = buf[0]
        
    if command == 1:    # Response core id
        check_len(buf, 2)
        st = buf[1]
        print(f"<core_id={st}>")
    elif command == 2:  # Response config string (null-terminated)
        print(f"<config_string:{buf[1:].decode('ascii', errors='ignore')}>")
    elif command == 3:  # Response joypad state
        check_len(buf, 5)
        st = buf[1:5]
        print(f"<joypad_state:{st.hex()}>")
    elif command == 4:  # floppy write request
        check_len(buf, 515)
        drive_sector = int.from_bytes(buf[1:3], 'big')
        drive = drive_sector >> 15
        sector = drive_sector & 0x7FFF
        data = buf[3:515]
        print(f"<floppy_write:{drive}:{sector}:{data[:4].hex()}...>")
    elif command == 5:  # floppy read request
        check_len(buf, 3)
        drive_sector = int.from_bytes(buf[1:3], 'big')
        drive = drive_sector >> 15
        sector = drive_sector & 0x7FFF
        print(f"<floppy_read:{drive}:{sector}>")
    else:
        print(f"{chr(command[0])}", end="")

def download_rom():
    ser.write(b'\x06\x01')   # Start loading ROM
    CHUNK_SIZE = 8 * 1024  # 8KB chunks
    with open(rom, 'rb') as f:
        while True:
            chunk = f.read(CHUNK_SIZE)
            if not chunk:
                break
            # Send command 0x07 followed by 3-byte length (MSB first)
            length = len(chunk)
            ser.write(b'\x07' + bytes([(length >> 16) & 0xFF, (length >> 8) & 0xFF, length & 0xFF]))
            # Send the chunk data
            ser.write(chunk)
    ser.write(b'\x06\x00')  # Signal loading complete
    ser.write(b'\x08\x00')  # Set overlay state to 0

if args.rom:
    download_rom()
    print("ROM loaded")
    sys.exit(0)

while True:
    if not args.fpga:
        handle_bl616_command()
    else:
        handle_fpga_command()
    