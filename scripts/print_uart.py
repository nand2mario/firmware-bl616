#!/usr/bin/python3

import csv
import sys

def is_visible_char(char_code):
    """Check if a character code represents a visible character or newline."""
    return char_code == 0x0A or (char_code >= 0x20 and char_code <= 0x7E)

def process_csv(file_path):
    """Process CSV file and print visible characters."""
    try:
        with open(file_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if len(row) >= 3:  # Ensure there are at least 3 columns
                    try:
                        # Get hex value from third column and convert to integer
                        char_code = int(row[2], 16)
                        if is_visible_char(char_code):
                            # Convert to character and print
                            print(chr(char_code), end='')
                    except ValueError:
                        # Skip if hex conversion fails
                        continue
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
    except Exception as e:
        print(f"Error processing file: {str(e)}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 print_uart.py <csv_file>")
    else:
        process_csv(sys.argv[1])


