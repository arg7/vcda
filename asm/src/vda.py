import sys
import re
from ISA import INSTRUCTION_MAP, REGISTERS, BRANCH_CONDITIONS, ALU_OPERATIONS, ALU_DATA_TYPES

# Import the tested and implemented functions
from alloc_parser import parse_array_declaration, encode_hex
from serialize import serialize_values

def parse_instruction(line, labels, current_address):
    # Match the instruction and operand (including @labels)
    match = re.match(r'(\w+)\s+(@?\w+)', line)  # Updated regex to handle @labels
    if match:
        instr, operand = match.groups()
        opcode = INSTRUCTION_MAP.get(instr, 0x0)
        
        # Handle specific instruction types
        if instr == 'RS':
            reg = REGISTERS.get(operand, 0)
            return f"{opcode:01X}{reg:01X}"
        elif instr == 'CS':
            cond = BRANCH_CONDITIONS.get(operand, 0)
            return f"{opcode:01X}{cond:01X}"
        elif instr == 'ADT':
            dtype = ALU_DATA_TYPES.get(operand, 0)
            return f"{opcode:01X}{dtype:01X}"
        elif instr == 'ALU':
            operation = ALU_OPERATIONS.get(operand, 0)
            return f"{opcode:01X}{operation:01X}"
        elif instr == 'LI':
            # LI instruction: opcode (2) + immediate value (4 bits)
            return f"{opcode:01X}{int(operand, 16):01X}"
        elif instr == 'JMP' or instr == 'CALL':
            # Handle labels for JMP and CALL
            if operand.startswith('@'):
                label = operand[1:]
                if label in labels:
                    offset = labels[label] - current_address - 1  # Relative to next instruction
                    return f"{opcode:01X}{offset & 0xF:01X}"  # 4-bit signed offset
            else:
                offset = int(operand, 16)
                return f"{opcode:01X}{offset & 0xF:01X}"
        else:
            return f"{opcode:02X}{int(operand, 16):02X}"
    return None

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

def assemble(input_file, output_file):
    # First pass: Collect labels and their addresses
    labels = {}
    current_address = 0
    with open(input_file, 'r') as infile:
        for line in infile:
            line = line.strip()
            if line.endswith(':'):  # Label definition
                label = line[:-1]
                labels[label] = current_address
            elif line.startswith('alloc'):
                # Alloc directives take up space based on the data
                data = parse_alloc(line)
                if data:
                    current_address += len(data)
            elif line.startswith(('JMP', 'RS', 'LI', 'CS', 'ADT', 'ALU')):
                current_address += 1  # Each instruction is 1 byte

    # Second pass: Generate the hex output
    current_address = 0
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            line = line.strip()
            if line.endswith(':'):  # Skip label definitions
                continue
            elif line.startswith('alloc'):
                data = parse_alloc(line)
                if data:
                    outfile.write(encode_hex(data) + ' ')
                    current_address += len(data)
            elif line.startswith(('JMP', 'RS', 'LI', 'CS', 'ADT', 'ALU')):
                hex_code = parse_instruction(line, labels, current_address)
                if hex_code:
                    outfile.write(hex_code + ' ')
                    current_address += 1
            elif line.startswith('@'):
                # Label reference, skip for now
                continue

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python vda.py <input_file.asm>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = input_file.replace('.asm', '.hex')
    
    assemble(input_file, output_file)
    print(f"Assembly completed. Output written to {output_file}")