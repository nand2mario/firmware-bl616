#!/usr/bin/python3

import sys

def process_file(file_path):
    total_characters = 0

    with open(file_path, 'r') as file:
        for line in file:
            if not line.startswith('/'):
                total_characters += len(line.strip())

    return total_characters

def fs_to_bin(fs_file_path, bin_file_path):
    with open(fs_file_path, 'r') as fs_file:
        with open(bin_file_path, 'wb') as bin_file:
            for line in fs_file:
                if not line.startswith('/'):
                    binary_string = line.strip()
                    for i in range(0, len(binary_string), 8):
                        byte = binary_string[i:i+8]
                        if len(byte) == 8:
                            bin_file.write(int(byte, 2).to_bytes(1, byteorder='big'))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <fs_file> <bin_file>")
        sys.exit(1)

    file_path = sys.argv[1]
    total_characters = process_file(file_path)
    print(f"Total number of characters: {total_characters}")
    print(f"Total bytes: {total_characters // 8}")

    if (len(sys.argv) == 3):
        bin_file_path = sys.argv[2]
        fs_to_bin(file_path, bin_file_path)
