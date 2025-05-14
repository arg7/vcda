# ISA.py - Instruction Set Architecture mappings for VD architecture

# Instruction opcode mapping (4-bit)
INSTRUCTION_MAP = {
    'NOP': 0x0,
    'RS': 0x1,
    'NS': 0x2,
    'LI': 0x3,
    'JMP': 0x4,
    'CALL': 0x5,
    'PUSH': 0x6,
    'POP': 0x7,
    'ALU': 0x8,
    'INT': 0x9,
    'IN': 0xA,
    'OUT': 0xB,
    'OP2': 0xC,
    'OP4': 0xD,
    'OP8': 0xE,
    'EXT': 0xF
}

# Mapping for NOP, RET, and IRET operands
NOP_EXT = {
    'RET': 0x01,
    'IRET': 0x02,
    'INC': 0x03,
    'DEC': 0x04,
    'NOT': 0x05,
}

# Register mappings
REGISTERS = {
    # Long forms
    'R.A': 0x0,
    'R.B': 0x1,
    'R.C': 0x2,
    'R.D': 0x3,
    'R.E': 0x4,
    'R.F': 0x5,
    'R.G': 0x6,

    'R.ALU_IO_CFG': 12,
    'R.ALU_MODE_CFG': 13,
    'R.ALU_VR_STRIDES': 14,
    'R.BRANCH_CTRL': 15,

    'R.ITBP': 252,
    'R.BP': 253, 
    'R.SP': 254,
    'R.IP': 255,

    #Nibbles in R.ALU_IO_CFG register
    'N.ARG1': 0,
    'N.ARG2': 2,
    'N.DST': 4,
    'N.NS': 6,

    #Nibbles in ALU_MODE_CFG register
    'M.ADT': 0,
    'M.VL': 2,
    'M.ST_DST': 4,

    #Nibbles in ALU_STRIDES register
    'S.ST_ARG1': 0,
    'S.ST_ARG2': 4,

    #Nibbles in BRANCH_CTRL register
    'B.BCS': 0,
    'B.ST_JMP': 2
}


# Branch Condition Selector mapping (4-bit) with short and long forms
BRANCH_CONDITIONS = {
    # Always
    'ALWAYS': 0x0,
    
    # Zero conditions
    'Z': 0x1,
    'Zero': 0x1,
    
    # Not Zero conditions
    'NZ': 0x2,
    'Not_Zero': 0x2,
    
    # Greater conditions
    'G': 0x3,
    'Greater': 0x3,
    
    # Greater or Equal conditions
    'GE': 0x4,
    'Greater_Or_Equal': 0x4,
    
    # Less conditions
    'L': 0x5,
    'Less': 0x5,
    
    # Less or Equal conditions
    'LE': 0x6,
    'Less_Or_Equal': 0x6,
    
    # Carry conditions
    'C': 0x7,
    'Carry': 0x7,
    
    # Not Carry conditions
    'NC': 0x8,
    'Not_Carry': 0x8,
    
    # Sign conditions
    'S': 0x9,
    'Sign': 0x9,
    
    # Not Sign conditions
    'NS': 0xA,
    'Not_Sign': 0xA,
    
    # Overflow conditions
    'O': 0xB,
    'Overflow': 0xB,
    
    # Not Overflow conditions
    'NO': 0xC,
    'Not_Overflow': 0xC,
    
    # Parity Even conditions
    'PE': 0xD,
    'Parity_Even': 0xD,
    
    # Parity Odd conditions
    'PO': 0xE,
    'Parity_Odd': 0xE,
    
    # Interrupt conditions
    'I': 0xF,
    'Interrupt': 0xF
}


# ALU Operation Mode Selector mapping (4-bit)
ALU_OPERATIONS = {
    'ADD': 0x0,
    'SUB': 0x1,
    'AND': 0x2,
    'OR': 0x3,
    'XOR': 0x4,
    'SHL': 0x5,
    'SHR': 0x6,
    'MUL': 0x7,
    'DIV': 0x8,
    'LOOKUP': 0x9,
    'LOAD': 0xA,
    'STORE': 0xB
}


# ALU Data Type Selector mapping (4-bit)
ALU_DATA_TYPES = {
    'u8': 0x0,      # 8-bit unsigned integer
    'i8': 0x1,      # 8-bit signed integer
    'u16': 0x2,     # 16-bit unsigned integer
    'i16': 0x3,     # 16-bit signed integer
    'u32': 0x4,     # 32-bit unsigned integer
    'i32': 0x5,     # 32-bit signed integer
    'u64': 0x6,     # 64-bit unsigned integer
    'i64': 0x7,     # 64-bit signed integer
    'f16': 0x8,     # 16-bit floating-point
    'f32': 0x9,     # 32-bit floating-point
    'f64': 0xA,     # 64-bit floating-point
    'u1': 0xB,      # 1-bit boolean
    'u4': 0xC,      # 4-bit unsigned integer
    'i4': 0xD,      # 4-bit integer
    'fp8': 0xE,     # 8-bit floating-point
}

FMT = {
    'RAW': 0x0,
    'DEC': 0x1,
    'HEX': 0x2,
    'BIN': 0x3,
    'FPE': 0x4,
    'FP0': 0x5,
    'FP2': 0x6,
    'FP4': 0x7,
}

IO = {
    'stdin': 0x0,
    'stdout': 0x0,
    'stderr': 0x1,
}

ISA_map = {
    'INST': INSTRUCTION_MAP,
    'INSTRUCTION_MAP': INSTRUCTION_MAP,
    'R': REGISTERS,
    'REGISTERS': REGISTERS,
    'ADT': ALU_DATA_TYPES,
    'ALU_DATA_TYPES': ALU_DATA_TYPES,
    'ALU': ALU_OPERATIONS,
    'ALU_OPERATIONS': ALU_OPERATIONS,
    'BCS': BRANCH_CONDITIONS,
    'BRANCH_CONDITIONS': BRANCH_CONDITIONS,
    'IO': IO,
    'FMT': FMT,
}

# Helper function to perform lookup and raise ValueError if not found
def lookup(mapping, key, error_message):
    if key in mapping:
        return mapping[key]
    else:
        raise ValueError(error_message)

def isa_lookup(key):
    if key in ISA_map:
        return ISA_map[key]
    else:
        raise ValueError("ISA error: '{key}' not defined")

# Print the entire ISA for debugging
def print_isa():
    print("=== Instruction Mappings ===")
    for name, value in INSTRUCTION_MAP.items():
        print(f"{name}: {value:01X}")
    
    print("\n=== Register Mappings ===")
    for name, value in REGISTERS.items():
        print(f"{name}: {value:01X}")
    
    print("\n=== Branch Condition Mappings ===")
    for name, value in BRANCH_CONDITIONS.items():
        print(f"{name}: {value:01X}")
    
    print("\n=== ALU Operation Mappings ===")
    for name, value in ALU_OPERATIONS.items():
        print(f"{name}: {value:01X}")
    
    print("\n=== ALU Data Type Mappings ===")
    for name, value in ALU_DATA_TYPES.items():
        print(f"{name}: {value:01X}")

# Example usage
if __name__ == "__main__":
    print_isa()