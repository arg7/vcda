import sys
import re
from ISA import INSTRUCTION_MAP, REGISTERS, BRANCH_CONDITIONS, ALU_OPERATIONS, ALU_DATA_TYPES, FMT, IO, lookup, isa_lookup
from alloc_parser import parse_constant, parse_array_declaration, encode_hex 
from serialize import serialize_values
from opcode_parser import OpcodeParser, OPCODE_PARAMS  # Adjust import based on your file structure


def serialize_opcode(opcode, labels):

    return bytes([(opcode << 4) | (arg & 0xF)])  # Mask to 4 bits


def parse_instruction(parser, line, labels, current_address, line_number):
    
    # Strip comments (everything after '//')
    line = line.split('//')[0].strip()
    
    # Skip empty lines or lines with only whitespace
    if not line:
        return None
    
    opcode = parser.parse(line)

    return serialize_opcode(opcode)

def parse_alloc(line):
    # Extract the array declaration part
    match = re.match(r'alloc\s+(.+)', line)
    if match:
        array_decl = match.group(1)
        # Parse the array declaration using the tested function
        t, l, v = parse_array_declaration(array_decl)
        # Serialize and encode the values using the tested functions
        return serialize_values(t, v)
    return None

byte_count = 0  # Global variable to track bytes

def format_hex(outfile, binary_data):
    global byte_count
    for i in range(0, len(binary_data), 16 - (byte_count % 16)):
        chunk = binary_data[i:i + 16 - (byte_count % 16)]
        byte_count += len(chunk)
        s = chunk.hex(' ') + ' '
        outfile.write(s)
        if byte_count % 16 == 0:
            outfile.write('\n')

def assemble(parser, input_file, output_file):
    # First pass: Collect labels and their addresses
    labels = {}
    current_address = 0
    with open(input_file, 'r') as infile:
        for line_number, line in enumerate(infile, start=1):  # Track line numbers
            line = line.strip()
            if line.endswith(':'):  # Label definition
                label = line[:-1]
                labels[label] = current_address
            elif line.startswith('alloc'):
                # Alloc directives take up space based on the data
                data = parse_alloc(line)
                if data:
                    current_address += len(data) // 2  # Each byte is 2 hex chars
            else:
                if parse_instruction(parser, line, None, current_address, line_number):
                    current_address += 1  # Each instruction is 1 byte

    # Second pass: Generate the hex output
    current_address = 0
    with open(input_file, 'r') as infile, open(output_file, 'w') if output_file  else sys.stdout as outfile:
        for line_number, line in enumerate(infile, start=1):  # Track line numbers
            line = line.strip()
            if line.endswith(':'):  # Skip label definitions
                continue
            elif line.startswith('alloc'):
                data = parse_alloc(line)
                if data:
                    format_hex(outfile, data)
                    current_address += len(data)
            elif line.startswith('@'):
                # Label reference, skip for now
                continue
            else:
                code = parse_instruction(parser, line, labels, current_address, line_number)
                if code:
                    format_hex(outfile, code)
                    current_address += 1

if __name__ == "__main__":
    #if len(sys.argv) != 2:
    #    print("Usage: python vda.py <input_file.asm>")
    #    sys.exit(1)
    
    input_file = 'asm/tests/basic_parsing.asm' #sys.argv[1]
    output_file = None #input_file.replace('.asm', '.hex')
    
    parser = OpcodeParser()

    assemble(parser, input_file, output_file)
    if output_file:
        print(f"Assembly completed. Output written to {output_file}")