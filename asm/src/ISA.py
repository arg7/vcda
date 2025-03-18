# ISA.py - Instruction Set Architecture mappings for VD architecture

# Instruction opcode mapping (4-bit)
INSTRUCTION_MAP = {
    'NOP': 0x0,
    'RET': 0x0,
    'IRET': 0x0,
    'SETC': 0x0,
    'CLSC': 0x0,
    'INC': 0x0,
    'DEC': 0x0,
    'NOT': 0x0,
    'CMP': 0x0,
    'FMT': 0x0,
    'RS': 0x1,
    'NS': 0x2,
    'LI': 0x3,
    'LIS': 0x4,
    'ALU': 0x5,
    'JMP': 0x6,
    'CALL': 0x7,
    'PUSH': 0x8,
    'POP': 0x9,
    'INT': 0xA,
    'IN': 0xB,
    'OUT': 0xC
}


# Register mapping (4-bit) with short and long forms
REGISTERS = {
    # Short forms
    'R0': 0x0,
    'R1': 0x1,
    'R2': 0x2,
    'R3': 0x3,
    'R4': 0x4,
    'R5': 0x5,
    'R6': 0x6,
    'R7': 0x7,
    'R8': 0x8,
    'R9': 0x9,
    'R10': 0xA,
    'R11': 0xB,
    'R12': 0xC,
    'R13': 0xD,
    'R14': 0xE,
    'R15': 0xF,
    
    # Long forms
    'R.A': 0x0,
    'R.B': 0x1,
    'R.C': 0x2,
    'R.D': 0x3,
    'R.E': 0x4,
    'R.F': 0x5,
    'R.G': 0x6,
    'R.SIMD_CTRL': 0x7,  # R4
    'R.SIMD_STRIDE': 0x8,  # R5
    'R.ITBP': 0x9,  # R9
    'R.ICTRL': 0xA,  # R10
    'R.F': 0xB,  # R11
    'R.JMP_STRIDE': 0xC,  # R12
    'R.JS': 0xC,
    'R.BP': 0xD,  # R13
    'R.SP': 0xE,  # R14
    'R.IP': 0xF,  # R15

    #Nibbles in Flags register
    'N.NS': 0,
    'N.RS': 1,
    'N.SRC': 2,
    'N.DST': 3,
    'N.BCS': 4,
    'N.ADT': 5,
    'N.CZSV': 6,
    'N.PI': 7,

    #Bits in nibbles
    'B.C': 1,
    'B.Z': 2,
    'B.S': 4,
    'B.V': 8,
    'B.P': 1,
    'B.I': 2
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
    'SAR': 0x7,
    'MUL': 0x8,
    'DIV': 0x9,
    'LOOKUP': 0xA,
    'LOAD': 0xB,
    'STORE': 0xC
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
    'i4': 0xC,      # 4-bit integer
    'fp4': 0xD,     # 4-bit floating-point
    'fp8': 0xE      # 8-bit floating-point
}

FMT = {
    'WORD': 0x0,
    'BYTE': 0x1,
    'NIBBLE': 0x2
}

IO = {
    'tty0': 0x0,
    'tty1': 0x1
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
    'IO': IO
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