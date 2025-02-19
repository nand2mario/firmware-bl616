#!/usr/bin/python3

import csv
import sys

# Define the TAP state transitions
transitions = {
    'Test-Logic-Reset': {0: 'Run-Test/Idle', 1: 'Test-Logic-Reset'},
    'Run-Test/Idle': {0: 'Run-Test/Idle', 1: 'Select-DR-Scan'},
    'Select-DR-Scan': {0: 'Capture-DR', 1: 'Select-IR-Scan'},
    'Capture-DR': {0: 'Shift-DR', 1: 'Exit1-DR'},
    'Shift-DR': {0: 'Shift-DR', 1: 'Exit1-DR'},
    'Exit1-DR': {0: 'Pause-DR', 1: 'Update-DR'},
    'Pause-DR': {0: 'Pause-DR', 1: 'Exit2-DR'},
    'Exit2-DR': {0: 'Shift-DR', 1: 'Update-DR'},
    'Update-DR': {0: 'Run-Test/Idle', 1: 'Select-DR-Scan'},
    'Select-IR-Scan': {0: 'Capture-IR', 1: 'Test-Logic-Reset'},
    'Capture-IR': {0: 'Shift-IR', 1: 'Exit1-IR'},
    'Shift-IR': {0: 'Shift-IR', 1: 'Exit1-IR'},
    'Exit1-IR': {0: 'Pause-IR', 1: 'Update-IR'},
    'Pause-IR': {0: 'Pause-IR', 1: 'Exit2-IR'},
    'Exit2-IR': {0: 'Shift-IR', 1: 'Update-IR'},
    'Update-IR': {0: 'Run-Test/Idle', 1: 'Select-DR-Scan'},
}

def format_bits(bit_str, allow_hex=False):
    if not bit_str:
        return '0x0'
    if len(bit_str) <= 64 and allow_hex:
        # Convert to hex, LSB first
        num_bits = len(bit_str)
        reversed_bit_str = bit_str[::-1]
        hex_str = hex(int(reversed_bit_str, 2))
        return hex_str + ' (' + str(num_bits) + ' bits)'
    else:
        # Split into 128-bit lines
        lines = []
        for i in range(0, len(bit_str), 64):
            chunk = bit_str[i:i+64]
            lines.append(chunk)
        return '\n'.join(lines)

def process_shift_data(time, tdi_bits, tdo_bits, reg_type, allow_hex=False):
    tdi_str = ''.join(str(b) for b in tdi_bits)
    tdo_str = ''.join(str(b) for b in tdo_bits)    
    print(f"  TDI: {format_bits(tdi_str, allow_hex)}")
    print(f"  TDO: {format_bits(tdo_str, allow_hex)}")

def main():
    current_state = 'Test-Logic-Reset'
    prev_tck = 0
    dr_tdi = []
    dr_tdo = []
    ir_tdi = []
    ir_tdo = []
    
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <jtag_data.csv>")
        print(f'Use tdi_compare.py to compare TDI to fs file')
        sys.exit(1)
    
    csvfile = sys.argv[1]

    with open(csvfile, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0].startswith(';'):
                continue
            break  # Exit after skipping comment lines
        next(reader)  # Skip header
        allowHex = True
        for row in reader:
            # Parse the row
            time = float(row[0])
            tdi = int(row[1])
            tck = int(row[2])
            tdo = int(row[3])
            tms = int(row[4])
            
            # Check for rising edge (0 -> 1)
            if prev_tck == 0 and tck == 1:
                previous_state = current_state
                new_state = transitions[previous_state][tms]
                
                # Capture TDI and TDO if previous state was Shift-DR or Shift-IR
                if previous_state == 'Shift-DR':
                    dr_tdi.append(tdi)
                    dr_tdo.append(tdo)
                elif previous_state == 'Shift-IR':
                    ir_tdi.append(tdi)
                    ir_tdo.append(tdo)
                
                # Check if DR or IR buffer is >= 128 bits and output the buffered bits
                if len(dr_tdi) >= 64:
                    process_shift_data(time, dr_tdi, dr_tdo, 'DR')
                    dr_tdi, dr_tdo = [], []
                    allowHex = False
                if len(ir_tdi) >= 64:
                    process_shift_data(time, ir_tdi, ir_tdo, 'IR')
                    ir_tdi, ir_tdo = [], []
                    allowHex = False

                # Check if exiting Shift state
                if previous_state in ['Shift-DR', 'Shift-IR'] and new_state not in ['Shift-DR', 'Shift-IR']:
                    if previous_state == 'Shift-DR':
                        process_shift_data(time, dr_tdi, dr_tdo, 'DR', allowHex)
                        dr_tdi, dr_tdo = [], []
                    else:
                        process_shift_data(time, ir_tdi, ir_tdo, 'IR', allowHex)
                        ir_tdi, ir_tdo = [], []
                
                # Output state transition if changed
                if new_state != current_state:
                    print(f"{time:.8f} {new_state}")
                    if new_state == 'Shift-DR' or new_state == 'Shift-IR':
                        allowHex = True
                
                current_state = new_state
            
            prev_tck = tck

if __name__ == "__main__":
    main()