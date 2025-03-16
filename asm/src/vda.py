import sys
import re
from ISA import INSTRUCTION_MAP, REGISTERS, BRANCH_CONDITIONS, ALU_OPERATIONS, ALU_DATA_TYPES, lookup
from alloc_parser import parse_constant, parse_array_declaration, encode_hex 
from serialize import serialize_values

# Mapping for NOP, RET, and IRET operands
NO_OPERAND_INSTRUCTIONS = {
    'NOP': 0,
    'RET': 1,
    'IRET': 2
}

def parse_instruction(line, labels, current_address, line_number):
    # Match the instruction and operand (including @labels and signed numbers)
    match = re.match(r'(\w+)\s+([@+-]?\w+)', line)  # Updated regex to handle signed numbers
    if not match:
        raise ValueError(f"Syntax error on line {line_number}: '{line.strip()}'")
    
    instr, operand = match.groups()
    opcode = INSTRUCTION_MAP.get(instr, 0x0)
    
    # Check if operand is a number (starts with 0-9, +, or -)
    if operand[0].isdigit() or operand.startswith(('+', '-')):
        # Operand is a number, use parse_constant directly
        arg = parse_constant(operand)
    else:
        # Operand is not a number, handle based on instruction type
        if instr in NO_OPERAND_INSTRUCTIONS:
            # NOP, RET, or IRET
            arg = NO_OPERAND_INSTRUCTIONS[instr]
        elif instr in ['RS', 'PUSH', 'POP']:
            # Instructions with register operand (mnemonic)
            arg = lookup(REGISTERS, operand, f"Invalid register: '{operand}'")
        elif instr == 'ADT':
            # ADT: Data type (mnemonic)
            arg = lookup(ALU_DATA_TYPES, operand, f"Invalid data type: '{operand}'")
        elif instr == 'ALU':
            # ALU: Operation (mnemonic)
            arg = lookup(ALU_OPERATIONS, operand, f"Invalid ALU operation: '{operand}'")
        elif instr == 'CS':
            # CS: Condition (mnemonic)
            arg = lookup(BRANCH_CONDITIONS, operand, f"Invalid condition: '{operand}'")
        elif instr in ['JMP', 'CALL']:
            # JMP/CALL: Label or 4-bit offset
            if operand.startswith('@'):
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
    return f"{opcode:01X}{arg & 0xF:01X}"

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
            elif line.startswith(('JMP', 'RS', 'LI', 'CS', 'ADT', 'ALU')):
                current_address += 1  # Each instruction is 1 byte

    # Second pass: Generate the hex output
    current_address = 0
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line_number, line in enumerate(infile, start=1):  # Track line numbers
            line = line.strip()
            if line.endswith(':'):  # Skip label definitions
                continue
            elif line.startswith('alloc'):
                data = parse_alloc(line)
                if data:
                    outfile.write(encode_hex(data) + ' ')
                    current_address += len(data) // 2
            elif line.startswith(('JMP', 'RS', 'LI', 'CS', 'ADT', 'ALU')):
                hex_code = parse_instruction(line, labels, current_address, line_number)  # Pass line_number
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