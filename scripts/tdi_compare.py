import re
import sys
import difflib

#!/usr/bin/python3

cnt = 0

def extract_tdi_bits(lines):
    global cnt
    pattern = re.compile(r"^ *TDI:\s*([01][01]+)")
    bits = ""
    for line in lines:
        match = pattern.match(line)
        if match:
            bits += match.group(1)
            cnt=cnt+1
            # if cnt < 10:
            #     print(f"{bits}")
    return bits

def read_fs_bitstream(filepath):
    with open(filepath, "r") as f:
        lines = f.readlines()
    # Remove all comment lines starting with /
    non_comment_lines = [line for line in lines if not line.lstrip().startswith("/")]
    # Remove all whitespace to get a continuous bitstream
    return "".join("".join(non_comment_lines).split())

def main():
    if len(sys.argv) != 3:
        print("Usage: tdi_compare.py <text_file> <fs_file>")
        sys.exit(1)

    text_file = sys.argv[1]
    fs_file = sys.argv[2]

    with open(text_file, "r") as f:
        lines = f.readlines()
    tdi_bits = extract_tdi_bits(lines)
    fs_bits = read_fs_bitstream(fs_file)

    def break_bits(bits, width=64):
        return [bits[i:i+width] for i in range(0, len(bits), width)]

    tdi_bits = break_bits(tdi_bits)
    fs_bits = break_bits(fs_bits)

    if tdi_bits == fs_bits:
        print("No differences found.")
    else:
        print("Differences found:")
        diff = difflib.unified_diff(
            tdi_bits, fs_bits,
            fromfile="TDI_bits", tofile="FS_bits",
            lineterm=""
        )
        for line in diff:
            print(line)

if __name__ == "__main__":
    main()