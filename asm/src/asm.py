import sys
import re
from ISA import INSTRUCTION_MAP, REGISTERS, BRANCH_CONDITIONS, ALU_OPERATIONS, ALU_DATA_TYPES, FMT, IO, lookup, isa_lookup
from alloc_parser import parse_constant, parse_array_declaration, encode_hex 
from serialize import serialize_values
from opcode_parser import OpcodeParser, OPCODE_PARAMS  # Adjust import based on your file structure

def serialize_opcode(parsed_instruction: dict, labels: dict, current_address: int) -> bytes:
    """
    Serialize an instruction into bytes based on the parser output and ISA definitions.
    
    Args:
        parsed_instruction: Dictionary with 'label' (optional), 'opcode', and 'args'.
        labels: Dictionary mapping label names to their addresses.
        current_address: The address of the current instruction.
    
    Returns:
        bytes: The serialized instruction.
    
    Raises:
        ValueError: If a label is duplicated or undefined, or if an argument is invalid.
        NotImplementedError: If the opcode has multiple arguments (for now).
    """
    # Handle label definition
    if 'label' in parsed_instruction and parsed_instruction['label'] is not None:
        label = parsed_instruction['label']
        if label in labels:
            raise ValueError(f"Duplicate label: {label}")
        labels[label] = current_address

    opcode = parsed_instruction['opcode']
    args = parsed_instruction.get('args', {})

    # Helper function to resolve argument values
    def resolve_argument(arg: str) -> int:
        if arg.startswith('@'):
            label = arg[1:]
            if label not in labels:
                raise ValueError(f"Undefined label: {label}")
            # Calculate offset: target address - (current_address + 1)
            return labels[label] - current_address - 1
        elif arg in REGISTERS:
            return REGISTERS[arg]
        elif arg in BRANCH_CONDITIONS:
            return BRANCH_CONDITIONS[arg]
        elif arg.isdigit():
            return int(arg)
        else:
            raise ValueError(f"Invalid argument: {arg}")

    # Case 1: Opcodes in NOP_EXT (e.g., RET, IRET, INC, DEC, NOT)
    if opcode in NOP_EXT:
        opcode_value = NOP_EXT[opcode]
        if not args:
            # No arguments: one byte with NOP (0x0) as first nibble
            return bytes([0x0 << 4 | opcode_value])
        elif len(args) == 1:
            # One argument: use OP2 encoding
            arg_name, arg_value = next(iter(args.items()))
            value = resolve_argument(arg_value)
            if 0 <= value <= 15:  # Fits in 4 bits, but we'll use OP2 for consistency
                return bytes([(INSTRUCTION_MAP['OP2'] << 4) | opcode_value, value])
            else:
                # Assume 8-bit value for simplicity
                return bytes([(INSTRUCTION_MAP['OP2'] << 4) | opcode_value, value & 0xFF])
        else:
            raise NotImplementedError("Multiple arguments for NOP_EXT opcodes not implemented")

    # Case 2: Other opcodes
    opcode_value = INSTRUCTION_MAP[opcode]
    if not args:
        # No arguments: one byte
        return bytes([opcode_value << 4 | 0])
    elif len(args) == 1:
        # One argument
        arg_name, arg_value = next(iter(args.items()))
        value = resolve_argument(arg_value)
        
        # Handle signed offsets for JMP/CALL
        if opcode in ('JMP', 'CALL') and arg_name == 'ofs':
            # Offset is signed; check if it fits in 4 bits (-8 to 7)
            if -8 <= value <= 7:
                # Convert to 4-bit two's complement
                value &= 0xF  # Take lower 4 bits, preserving sign in 4-bit range
                return bytes([opcode_value << 4 | value])
            else:
                # Two bytes: OP2 prefix
                return bytes([(INSTRUCTION_MAP['OP2'] << 4) | opcode_value, value & 0xFF])
        # Handle registers or other arguments
        elif 0 <= value <= 15:
            # Fits in 4 bits
            return bytes([opcode_value << 4 | value])
        else:
            # Two bytes: OP2 prefix
            return bytes([(INSTRUCTION_MAP['OP2'] << 4) | opcode_value, value & 0xFF])
    else:
        # Multiple arguments: not implemented yet
        raise NotImplementedError("Opcodes with multiple arguments not implemented")
    

def parse_instruction(parser, line, labels, current_address, line_number):
    
    # Strip comments (everything after '//')
    line = line.split('//')[0].strip()
    
    # Skip empty lines or lines with only whitespace
    if not line:
        return None
    
    opcode = parser.parse(line)
    return serialize_opcode(opcode, labels, current_address)

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