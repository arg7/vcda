import sys
import re
from ISA import INSTRUCTION_MAP, REGISTERS, BRANCH_CONDITIONS, ALU_OPERATIONS, ALU_DATA_TYPES, FMT, IO, lookup, isa_lookup
from alloc_parser import parse_constant, parse_array_declaration, encode_hex 
from serialize import serialize_values

# Mapping for NOP, RET, and IRET operands
NO_OPERAND_INSTRUCTIONS = {
    'RET': 0x01,
    'IRET': 0x02,
    'INC': 0x03,
    'DEC': 0x04,
    'NOT': 0x05,
}

def combine_opcode_arg(opcode, arg):
    if arg < 0:
        arg = 16 + arg # two's complement for 4 bits

    return bytes([(opcode << 4) | (arg & 0xF)])  # Mask to 4 bits


def parse_instruction(line, labels, current_address, line_number):
    
    # Strip comments (everything after '//')
    line = line.split('//')[0].strip()
    
    # Skip empty lines or lines with only whitespace
    if not line:
        return None
    
    # Match the instruction and operand (including @labels and signed numbers)
    match = re.match(r'(\w+)\s+([@+-]?\w+(?:\.\w+)?)', line)
    if not match:
        raise ValueError(f"Syntax error on line {line_number}: '{line.strip()}'")
    
    instr, operand = match.groups()
    opcode = lookup(INSTRUCTION_MAP, instr, f"Invalid instruction: '{instr}'")
    
    # Check if operand is a number (starts with 0-9, +, or -)
    if operand[0].isdigit() or operand.startswith(('+', '-')):
        # Operand is a number, use parse_constant directly
        arg = parse_constant(operand)
    else:
        # Operand is not a number, handle based on instruction type
        if instr == 'FMT':
            arg = NO_OPERAND_INSTRUCTIONS[instr] + lookup(FMT, operand, f"Invalid chanel '{operand}'")
        if instr in NO_OPERAND_INSTRUCTIONS:
            # NOP, RET, or IRET
            arg = NO_OPERAND_INSTRUCTIONS[instr]
        elif instr in ['LI', 'LIS', 'IN', 'OUT']:
            # parse format MAP.val
            msg = f"Invalid value: '{operand}'"
            o = operand.split(".");
            if len(o) > 1:
                arg = lookup(isa_lookup(o[0]), o[1], msg)
            else:
                raise ValueError(msg)
        elif instr in ['NS','RS', 'PUSH', 'POP']:
            # Instructions with register operand (mnemonic)
            arg = lookup(REGISTERS, operand, f"Invalid register: '{operand}'")
        elif instr == 'ALU':
            # ALU: Operation (mnemonic)
            arg = lookup(ALU_OPERATIONS, operand, f"Invalid ALU operation: '{operand}'")
        elif instr in ['JMP', 'CALL']:
            # JMP/CALL: Label or 4-bit offset
            if not labels:
                arg = 0
            elif operand.startswith('@'):
                label = operand[1:]
                if label in labels:
                    arg = labels[label] - current_address - 1  # Relative to next instruction
                else:
                    raise ValueError(f"Label '{label}' not defined")  # Throw exception for unknown labels
            else:
                raise ValueError(f"Invalid operand for {instr}: '{operand}'")
        else:
            # Unknown instruction
            raise ValueError(f"Unknown instruction: '{instr}'")
    
    # Return the instruction as a single byte (opcode + operand)
    return combine_opcode_arg(opcode, arg)

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

def assemble(input_file, output_file):
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
                if parse_instruction(line, None, current_address, line_number):
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
                code = parse_instruction(line, labels, current_address, line_number)
                if code:
                    format_hex(outfile, code)
                    current_address += 1

if __name__ == "__main__":
    #if len(sys.argv) != 2:
    #    print("Usage: python vda.py <input_file.asm>")
    #    sys.exit(1)
    
    input_file = 'asm/tests/basic_parsing.asm' #sys.argv[1]
    output_file = None #input_file.replace('.asm', '.hex')
    
    assemble(input_file, output_file)
    if output_file:
        print(f"Assembly completed. Output written to {output_file}")