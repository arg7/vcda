import sys
import re
from ISA import INSTRUCTION_MAP, REGISTERS, BRANCH_CONDITIONS, ALU_OPERATIONS, ALU_DATA_TYPES, FMT, IO, lookup, isa_lookup, NOP_EXT
from alloc_parser import parse_constant, parse_array_declaration, encode_hex 
from opcode_parser import OpcodeParser, OPCODE_PARAMS  # Adjust import based on your file structure
import math

def is_in_signed_int_range(value, n_bits):
    """
    Check if a value is within the range of an N-bit signed integer.
    
    Args:
        value: The value to check (integer or float that will be converted to int).
        n_bits: The number of bits for the signed integer (e.g., 8, 16, 32).
        
    Returns:
        bool: True if the value is within the range, False otherwise.
        
    Raises:
        ValueError: If n_bits is less than 1 or if value cannot be converted to an integer.
    """
    if n_bits < 1:
        raise ValueError("Number of bits must be at least 1")
    
    try:
        value = int(value)
    except (ValueError, TypeError):
        raise ValueError("Value must be convertible to an integer")
    
    # Calculate the range for an N-bit signed integer
    min_value = -(2 ** (n_bits - 1))
    max_value = (2 ** (n_bits - 1)) - 1
    
    # Check if value is within the range
    return min_value <= value <= max_value

def serialize_signed_int_to_bytes(value, n_bits):
    """
    Serialize a signed integer to bytes in little-endian format for a given bit length.
    
    Args:
        value: The signed integer to serialize (integer or float convertible to int).
        n_bits: The number of bits for the signed integer (e.g., 8, 16, 32).
        
    Returns:
        bytes: The serialized integer in little-endian byte order.
        
    Raises:
        ValueError: If n_bits < 1, value is out of range, or value cannot be converted to int.
    """
    # Validate inputs
    if n_bits < 1:
        raise ValueError("Number of bits must be at least 1")
    
    try:
        value = int(value)
    except (ValueError, TypeError):
        raise ValueError("Value must be convertible to an integer")
    
    # Check if value is within the range for n_bits
    if not is_in_signed_int_range(value, n_bits):
        raise ValueError(f"Value {value} is out of range for {n_bits}-bit signed integer")
    
    # Calculate the number of bytes needed (ceiling of n_bits / 8)
    num_bytes = math.ceil(n_bits / 8)
    
    # Convert to bytes in little-endian format
    # For negative numbers, ensure sign extension to fill the required bytes
    return value.to_bytes(length=num_bytes, byteorder='little', signed=True)

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

    def _serialize_branch_instruction(opcode, resolved_args):
        """
        Helper function to serialize branch instructions (JMP, CALL).
        
        Args:
            opcode: The instruction opcode (e.g., 'JMP', 'CALL').
            resolved_args: Dictionary containing resolved arguments ('ofs', 'bcs').
            
        Returns:
            bytes: The serialized instruction in little-endian format.
            
        Raises:
            ValueError: If required arguments are missing or out of range.
            NotImplementedError: If large bcs/ofs combinations are not implemented.
        """
        if 'ofs' not in resolved_args:
            raise ValueError(f"{opcode} requires 'ofs' argument")
        ofs = resolved_args['ofs']
        
        if 'bcs' not in resolved_args:
            if -8 <= ofs <= 7:
                # 1-byte: 0xXo (X=opcode, o=ofs:signed u4)
                encoded_ofs = encode_signed(ofs, 4)
                return bytes([(INSTRUCTION_MAP[opcode] << 4) | encoded_ofs])
            else:
                raise ValueError(f"Offset too large for 1-byte {opcode}")
        else:
            bcs = resolved_args['bcs']
            if 0 <= bcs <= 15 and -8 <= ofs <= 7:
                # 2-byte: 0xCX (b << 4 | o) (X=opcode, b=bcs:u4, o=ofs:signed u4)
                byte0 = (0xC << 4) | INSTRUCTION_MAP[opcode]
                encoded_ofs = encode_signed(ofs, 4)
                byte1 = (bcs << 4) | encoded_ofs
                return bytes([byte0, byte1])
            elif 0 <= bcs <= 255 and is_in_signed_int_range(ofs, 16):
                # 3-byte: 0xDX (X=opcode, bcs:u8, ofs:signed 16-bit)
                byte0 = (0xD << 4) | INSTRUCTION_MAP[opcode]
                byte1 = bcs
                ofs_bytes = serialize_signed_int_to_bytes(ofs, 16)
                return bytes([byte0, byte1]) + ofs_bytes
            elif 0 <= bcs <= 255 and is_in_signed_int_range(ofs, 44):
                # 7-byte: 0xEX (X=opcode, bcs:u8, ofs:signed 44-bit)
                byte0 = (0xE << 4) | INSTRUCTION_MAP[opcode]
                byte1 = bcs
                ofs_bytes = serialize_signed_int_to_bytes(ofs, 44)
                return bytes([byte0, byte1]) + ofs_bytes
            
            # Larger sizes not implemented here
            raise NotImplementedError(f"Large bcs/ofs for {opcode} not implemented")
    
    # Helper function for stack instructions (PUSH, POP)
    def _serialize_stack_instruction(opcode, resolved_args):
        if 'reg' not in resolved_args:
            raise ValueError(f"{opcode} requires 'reg' argument")
        reg = resolved_args['reg']
        if 0 <= reg <= 15:
            # 1-byte: 0xXr (X=opcode, r=reg:u4)
            return bytes([(INSTRUCTION_MAP[opcode] << 4) | reg])
        elif 0 <= reg <= 255:
            # 2-byte: 0xCX r (X=opcode, r=reg:u8)
            byte0 = (0xC << 4) | INSTRUCTION_MAP[opcode]
            byte1 = reg & 0xFF
            return bytes([byte0, byte1])
        else:
            raise ValueError(f"Register {reg} out of range")
        # 4-byte format (cnt for PUSH, cnt+ofs for POP) not implemented
        raise NotImplementedError(f"4-byte format for {opcode} not implemented")

    # Helper function for register instructions (RS, NS)
    def _serialize_register_instruction(opcode, resolved_args):
        if 'val' not in resolved_args:
            raise ValueError(f"{opcode} requires argument")
        reg = resolved_args['val']
        if 0 <= reg <= 15:
            # 1-byte: 0xXr (X=opcode, r=reg:u4)
            return bytes([(INSTRUCTION_MAP[opcode] << 4) | reg])
        elif 0 <= reg <= 255:
            # 2-byte: 0xCX r (X=opcode, r=reg:u8)
            byte0 = (0xC << 4) | INSTRUCTION_MAP[opcode]
            byte1 = reg & 0xFF
            return bytes([byte0, byte1])
        else:
            raise ValueError(f"Register {reg} out of range")

    # Helper function for ALU instructions
    def _serialize_alu_instruction(opcode, resolved_args):
        if 'op' not in resolved_args:
            raise ValueError("ALU requires 'op' argument")
        op = resolved_args['op']
        if 'adt' not in resolved_args:
            if 0 <= op <= 15:
                # 1-byte: 0x8o (opcode=0x8, o=op:u4)
                return bytes([(INSTRUCTION_MAP['ALU'] << 4) | op])
            else:
                raise ValueError("Operation code too large for 1-byte ALU")
        else:
            adt = resolved_args['adt']
            if 0 <= op <= 15 and 0 <= adt <= 15:
                # 2-byte: 0xC8 (o << 4 | t) (o=op:u4, t=adt:u4)
                byte0 = (0xC << 4) | INSTRUCTION_MAP['ALU']
                byte1 = (op << 4) | adt
                return bytes([byte0, byte1])
            # Larger sizes not implemented here
            raise NotImplementedError("Large op/adt for ALU not implemented")

    # Helper function for IO instructions (INT, IN, OUT)
    def _serialize_io_instruction(opcode, resolved_args):
        arg_name = 'val' if opcode == 'INT' else 'ch'
        if arg_name not in resolved_args:
            raise ValueError(f"{opcode} requires '{arg_name}' argument")
        value = resolved_args[arg_name]
        if 0 <= value <= 15:
            # 1-byte: 0xXv (X=opcode, v=val/ch:u4)
            return bytes([(INSTRUCTION_MAP[opcode] << 4) | value])
        elif 0 <= value <= 255:
            # 2-byte: 0xCX v (X=opcode, v=val/ch:u8)
            byte0 = (0xC << 4) | INSTRUCTION_MAP[opcode]
            byte1 = value & 0xFF
            return bytes([byte0, byte1])
        else:
            raise ValueError(f"{arg_name.capitalize()} {value} out of range")
        # 4-byte format (IN/OUT only) not implemented
        raise NotImplementedError(f"4-byte format for {opcode} not implemented")

    # Helper function to encode signed values
    def encode_signed(value, bits):
        min_val, max_val = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
        if not (min_val <= value <= max_val):
            raise ValueError(f"Value {value} out of range for {bits}-bit signed")
        return value & ((1 << bits) - 1)

    # Handle label definition
    if 'label' in parsed_instruction and parsed_instruction['label']:
        label = parsed_instruction['label']
        labels[label] = current_address

    opcode = parsed_instruction['opcode']
    args = parsed_instruction.get('args', {})

    if opcode == 'ALLOC':
        return args

    if args:
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
            elif key in ('reg', 'ns') and value in REGISTERS:
                resolved_args[key] = REGISTERS[value]
            elif key == 'op' and value in ALU_OPERATIONS:
                resolved_args[key] = ALU_OPERATIONS[value]
            elif key == 'adt' and value in ALU_DATA_TYPES:
                resolved_args[key] = ALU_DATA_TYPES[value]
            elif key == 'bcs' and value in BRANCH_CONDITIONS:
                resolved_args[key] = BRANCH_CONDITIONS[value]
            elif key in ('val', 'cnt', 'ch'):
                resolved_args[key] = isa_lookup(value)
            else:
                raise ValueError(f"Invalid argument: {key}={value}")

    if not opcode:
        return bytes([]);

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

    elif opcode in ('RS', 'NS'):
        return _serialize_register_instruction(opcode, resolved_args)

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

    elif opcode in ('JMP', 'CALL'):
            return _serialize_branch_instruction(opcode, resolved_args)
    
    elif opcode in ('PUSH', 'POP'):
        return _serialize_stack_instruction(opcode, resolved_args)
    
    elif opcode == 'ALU':
        return _serialize_alu_instruction(opcode, resolved_args)
    
    elif opcode in ('INT', 'IN', 'OUT'):
        return _serialize_io_instruction(opcode, resolved_args)
    
    else:
        raise NotImplementedError(f"Serialization for opcode {opcode} not implemented")    

def parse_instruction(parser, line, line_number):
    
    # Strip comments (everything after '//')
    line = line.split('//')[0].strip()
    
    # Skip empty lines or lines with only whitespace
    if not line:
        return None
    
    o = parser.parse(line)
    return o

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
    list = []

    with open(input_file, 'r') as infile:
        for line_number, line in enumerate(infile, start=1):  # Track line numbers
            o = parse_instruction(parser, line, line_number)
            if o:
                if o['label']:
                    labels[o['label']] = current_address
                list.append(o)
                current_address += o['size']


    # Next passes: Generate binary, until JMP/CALLs stabilize
    binary_size = -1;
    while binary_size != current_address : 
        
        binary_size = current_address
        current_address = 0
        
        for o in list:

            o['obj'] = serialize_opcode(o, labels, current_address)
            if o['obj']:
                current_address += len(o['obj'])

    with open(output_file, 'w') if output_file  else sys.stdout as outfile:
        for o in list:
            if o['obj']:
                format_hex(outfile, o['obj'])
# done assembly()

if __name__ == "__main__":
    #if len(sys.argv) != 2:
    #    print("Usage: python vda.py <input_file.asm>")
    #    sys.exit(1)
    
    input_file = '../tests/basic_parsing.asm' #sys.argv[1]
    output_file = input_file.replace('.asm', '.hex')
    
    parser = OpcodeParser()

    assemble(parser, input_file, output_file)
    if output_file:
        print(f"Assembly completed. Output written to {output_file}")