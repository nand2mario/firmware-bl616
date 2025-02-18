#!/usr/bin/python3
import csv

def parse_uart_log(csv_file):
    # Read CSV and extract byte values
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        bytes_list = [row[2] for row in reader]

    # Convert hex strings to integers
    bytes_data = [int(byte, 16) for byte in bytes_list]
    
    # Find the start sequence 0x04, 0x00, 0x1B
    start_index = 0
    for i in range(len(bytes_data) - 2):
        if bytes_data[i] == 0x04 and bytes_data[i+1] == 0x00 and bytes_data[i+2] == 0x1B:
            start_index = i
            break
    
    if start_index == 0:
        print("Start sequence not found!")
        return

    # Process commands
    i = start_index
    while i < len(bytes_data):
        cmd = bytes_data[i]
        
        if cmd == 0x04:  # Cursor position command
            if i+2 >= len(bytes_data):
                print("Incomplete cursor position command")
                break
            x = bytes_data[i+1]
            y = bytes_data[i+2]
            print(f"Cursor Position: X={x}, Y={y}")
            i += 3
            
        elif cmd == 0x05:  # Print command
            i += 1
            string = ""
            while i < len(bytes_data) and bytes_data[i] != 0x00:
                string += chr(bytes_data[i])
                i += 1
            if i < len(bytes_data) and bytes_data[i] == 0x00:
                print(f"Print: '{string}'")
                i += 1
                
        else:
            print(f"Unknown command: 0x{cmd:02X}")
            i += 1

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: verify_uart_screen.py <uart_log.csv>")
        sys.exit(1)
    
    parse_uart_log(sys.argv[1])

