import sys
import re
from ISA import INSTRUCTION_MAP, REGISTERS, BRANCH_CONDITIONS, ALU_OPERATIONS, ALU_DATA_TYPES, FMT, IO, lookup, isa_lookup
from alloc_parser import parse_constant, parse_array_declaration, encode_hex 
from serialize import serialize_values
from opcode_parser import OpcodeParser, OPCODE_PARAMS  # Adjust import based on your file structure

def serialize_opcode(parsed_instruction, labels, current_address):
    """
    Serializes a parsed instruction into a byte sequence based on the ISA specification.
    
    Args:
        parsed_instruction (dict): Dictionary with 'label', 'opcode', and 'args'.
        labels (dict): Dictionary mapping label names to their addresses.
        current_address (int): Current instruction address for offset calculations.
    
    Returns:
        bytes: Serialized instruction as a byte sequence.
    
    Raises:
        ValueError: If arguments are invalid or labels are undefined/duplicate.
        NotImplementedError: If serialization for an opcode/size is not yet implemented.
    """
    # Handle label definition
    if 'label' in parsed_instruction and parsed_instruction['label']:
        label = parsed_instruction['label']
        if label in labels:
            raise ValueError(f"Duplicate label: {label}")
        labels[label] = current_address

    opcode = parsed_instruction['opcode']
    args = parsed_instruction.get('args', {})

    # Resolve arguments
    resolved_args = {}
    for key, value in args.items():
        if key == 'ofs' and isinstance(value, str):
            if value.startswith('@'):
                label = value[1:]
                if label not in labels:
                    raise ValueError(f"Undefined label: {label}")
                resolved_args[key] = labels[label] - current_address - 1
            else:
                resolved_args[key] = int(value)
        elif key == 'reg' and value in REGISTERS:
            resolved_args[key] = REGISTERS[value]
        elif key == 'bcs' and value in BRANCH_CONDITIONS:
            resolved_args[key] = BRANCH_CONDITIONS[value]
        elif key in ('val', 'cnt') and isinstance(value, (str, int)):
            resolved_args[key] = int(value)
        else:
            raise ValueError(f"Invalid argument: {key}={value}")

    # Helper function to encode signed values
    def encode_signed(value, bits):
        min_val, max_val = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
        if not (min_val <= value <= max_val):
            raise ValueError(f"Value {value} out of range for {bits}-bit signed")
        return value & ((1 << bits) - 1)

    # Serialize based on opcode
    if opcode in NOP_EXT:
        sub_opcode = NOP_EXT[opcode]
        if not resolved_args:
            # 1-byte: 0x0s (s = sub-opcode:u4)
            return bytes([sub_opcode])
        elif 'cnt' in resolved_args or 'reg' in resolved_args:
            arg_name = 'cnt' if 'cnt' in resolved_args else 'reg'
            arg_value = resolved_args[arg_name]
            if 0 <= arg_value <= 15:
                # 2-byte: 0xC0 (s << 4 | arg:u4)
                byte0 = 0xC0  # OP2 prefix + 0x0
                byte1 = (sub_opcode << 4) | arg_value
                return bytes([byte0, byte1])
            # Larger sizes (4-byte, 8-byte) not implemented here
            raise NotImplementedError(f"Large arguments for {opcode} not implemented")
        else:
            raise ValueError(f"Invalid arguments for {opcode}")

    elif opcode == 'RS':
        if 'reg' not in resolved_args:
            raise ValueError("RS requires 'reg' argument")
        reg = resolved_args['reg']
        if 0 <= reg <= 15:
            # 1-byte: 0x1r (opcode=0x1, r=reg:u4)
            return bytes([(INSTRUCTION_MAP['RS'] << 4) | reg])
        elif 0 <= reg <= 255:
            # 2-byte: 0xC1 r (r=reg:u8)
            byte0 = (0xC << 4) | INSTRUCTION_MAP['RS']
            byte1 = reg & 0xFF
            return bytes([byte0, byte1])
        else:
            raise ValueError(f"Register {reg} out of range")

    elif opcode == 'LI':
        if 'val' not in resolved_args:
            raise ValueError("LI requires 'val' argument")
        val = resolved_args['val']
        if 'reg' not in resolved_args:
            if 0 <= val <= 15:
                # 1-byte: 0x3v (opcode=0x3, v=val:u4)
                return bytes([(INSTRUCTION_MAP['LI'] << 4) | val])
            else:
                raise ValueError("Value too large for 1-byte LI without reg")
        else:
            reg = resolved_args['reg']
            if 0 <= reg <= 15 and 0 <= val <= 15:
                # 2-byte: 0xC3 (r << 4 | v) (r=reg:u4, v=val:u4)
                byte0 = (0xC << 4) | INSTRUCTION_MAP['LI']
                byte1 = (reg << 4) | val
                return bytes([byte0, byte1])
            # Larger sizes not implemented here
            raise NotImplementedError(f"Large reg/val for LI not implemented")

    elif opcode == 'JMP':
        if 'ofs' not in resolved_args:
            raise ValueError("JMP requires 'ofs' argument")
        ofs = resolved_args['ofs']
        if 'bcs' not in resolved_args:
            if -8 <= ofs <= 7:
                # 1-byte: 0x4o (opcode=0x4, o=ofs:signed u4)
                encoded_ofs = encode_signed(ofs, 4)
                return bytes([(INSTRUCTION_MAP['JMP'] << 4) | encoded_ofs])
            else:
                raise ValueError("Offset too large for 1-byte JMP")
        else:
            bcs = resolved_args['bcs']
            if 0 <= bcs <= 15 and -8 <= ofs <= 7:
                # 2-byte: 0xC4 (b << 4 | o) (b=bcs:u4, o=ofs:signed u4)
                byte0 = (0xC << 4) | INSTRUCTION_MAP['JMP']
                encoded_ofs = encode_signed(ofs, 4)
                byte1 = (bcs << 4) | encoded_ofs
                return bytes([byte0, byte1])
            # Larger sizes not implemented here
            raise NotImplementedError(f"Large bcs/ofs for JMP not implemented")

    else:
        raise NotImplementedError(f"Serialization for opcode {opcode} not implemented")    

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