import unittest
import re
import codecs
import struct
from typing import List, Tuple, Union

def parse_array_declaration(input_string: str) -> Tuple[str, int, List[Union[int, float, str]]]:
    """
    Parses an array declaration string into type, size, and values.
    """
    match = re.match(r"(\w+)(?:\[(\d*)\])?(?:\s*=\s*(.*))?", input_string)
    if not match:
        return None

    type_ = match.group(1)
    size = int(match.group(2)) if match.group(2) else None
    values_str = match.group(3)

    if values_str is None:  # No '=' at all
        if size is None:
            size = 1
            values = [0]  # Initialize with a single zero if only type is given
        else:
            values = [0] * size  # Initialize with zeros if size is specified
    elif values_str:  # Values provided after '='
        if values_str.startswith("{") and values_str.endswith("}"):
            values_str = values_str[1:-1]
            values = [parse_constant(val.strip()) for val in values_str.split(",")]
            if size is None:
                size = len(values)
            else:
                # Pad with zeros if size is larger than the number of values
                values.extend([0] * (size - len(values)))
        elif values_str.startswith('"') and values_str.endswith('"'):
            values_str = codecs.decode(values_str[1:-1], 'unicode_escape')
            values = list(values_str)  # Convert string to list of characters
            if size is None:
                size = len(values) + 1  # +1 for null terminator
                values.append('\0')  # Add null terminator
            else:
                # Pad with zeros if size is larger than the string length
                values.extend(['\0'] * (size - len(values)))
        else:
            values = [parse_constant(values_str.strip())]
            if size is None:
                size = 1
            else:
                # Pad with zeros if size is larger than 1
                values.extend([0] * (size - 1))
    elif size is not None:  # '=' is present, no values after it, but size is specified
        values = [0] * size  # Initialize with zeros
    else:  # '=' is present but no values, and no size
        return None  # Syntax error: '=' with no value and no size

    return type_, size, values


def parse_constant(constant_str: str) -> Union[int, float, str]:
    """
    Parses a constant value (int, float, hex, binary, or char).
    """
    if constant_str.startswith("'") and constant_str.endswith("'"):
       return codecs.decode(constant_str[1:-1], 'unicode_escape') 
    elif constant_str.startswith("0x"):
        return int(constant_str, 16)
    elif constant_str.startswith("0b"):
        return int(constant_str, 2)
    elif "." in constant_str:
        return float(constant_str)
    else: 
        return int(constant_str)


def serialize_values(type_: str, values: List[Union[int, float, str]]) -> bytes:
    """
    Serializes values into bytes according to the specified type.
    Supports u1 (1-bit), (u)int4 (4-bit), (u)int8/16/32/64, float32/64, and strings.
    """
    if type_ == "u1":
        # Pack bools as 1-bit values (least significant bit first)
        bits = "".join(str(int(bool(v))) for v in reversed(values))  # Reverse to ensure LSB first
        # Pad to a multiple of 8 bits
        bits = bits.ljust((len(bits) + 7) // 8 * 8, '0')
        # Convert to bytes (big-endian)
        return int(bits, 2).to_bytes((len(bits) + 7) // 8, byteorder="big")
    elif type_.startswith(("i", "u")) and type_.endswith("4"):
        # Pack 4-bit integers
        bits = ""
        for v in values:
            if type_.startswith("u"):
                bits += f"{v & 0xF:04b}"  # Unsigned 4-bit
            else:
                bits += f"{v & 0xF:04b}"  # Signed 4-bit (same as unsigned for now)
        # Pad to a multiple of 8 bits
        bits = bits.ljust((len(bits) + 7) // 8 * 8, '0')
        # Convert to bytes
        return int(bits, 2).to_bytes((len(bits) + 7) // 8, byteorder="big")
    elif type_.startswith(("i", "u")):
        # Determine bit width and signedness
        if type_.startswith("u"):
            bit_width = int(type_[1:])  # Extract bit width for unsigned types (e.g., "uint16" -> 16)
        else:
            bit_width = int(type_[1:])  # Extract bit width for signed types (e.g., "int8" -> 8)
        is_signed = type_.startswith("i")
        # Pack integers
        return b"".join(v.to_bytes(bit_width // 8, byteorder="little", signed=is_signed) for v in values)
    elif type_.startswith("f"):
        # Determine bit width
        bit_width = int(type_[1:])  # Extract bit width (e.g., 32, 64)
        # Pack floats
        if bit_width == 32:
            return b"".join(struct.pack("<f", v) for v in values)
        elif bit_width == 64:
            return b"".join(struct.pack("<d", v) for v in values)
        else:
            raise ValueError(f"Unsupported float type: {type_}")
    elif type_ == "char":
        # Pack strings as UTF-8 bytes
        return "".join(values).encode("utf-8")
    else:
        raise ValueError(f"Unsupported type: {type_}")
    
def encode_hex(serialized: bytes, bytes_per_line: int = 16) -> str:
    """
    Encodes serialized bytes into a hex string with 16 hex numbers per line.
    """
    hex_str = serialized.hex()  # Convert bytes to hex string
    # Split hex string into chunks of 2 characters (1 byte)
    hex_chunks = [hex_str[i:i+2] for i in range(0, len(hex_str), 2)]
    # Group chunks into lines of `bytes_per_line`
    lines = []
    for i in range(0, len(hex_chunks), bytes_per_line):
        line = " ".join(hex_chunks[i:i+bytes_per_line])
        lines.append(line)
    return "\n".join(lines)
    

