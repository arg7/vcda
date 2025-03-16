import struct
from typing import List, Union

def serialize_values(type_: str, values: List[Union[int, float, str]]) -> bytes:
    if type_ == 'bool':
        # Pack bools into bits, 8 per byte
        bits = []
        for value in values:
            bits.append(1 if value else 0)
        # Pad with zeros to make a multiple of 8
        while len(bits) % 8 != 0:
            bits.append(0)
        # Pack bits into bytes
        byte_array = bytearray()
        for i in range(0, len(bits), 8):
            byte = 0
            for j in range(8):
                byte |= bits[i + j] << j
            byte_array.append(byte)
        return bytes(byte_array)
    
    elif type_ in ['int4', 'uint4']:
        # Pack 4-bit integers into bytes, 2 per byte
        byte_array = bytearray()
        for i in range(0, len(values), 2):
            byte = 0
            if i < len(values):
                byte |= (values[i] & 0x0F) << 4
            if i + 1 < len(values):
                byte |= values[i + 1] & 0x0F
            byte_array.append(byte)
        return bytes(byte_array)
    
    elif type_ in ['int8', 'uint8']:
        return struct.pack(f'<{len(values)}B', *values)
    
    elif type_ in ['int16', 'uint16']:
        return struct.pack(f'<{len(values)}H', *values)
    
    elif type_ in ['int32', 'uint32']:
        return struct.pack(f'<{len(values)}I', *values)
    
    elif type_ in ['int64', 'uint64']:
        return struct.pack(f'<{len(values)}Q', *values)
    
    elif type_ == 'float32':
        return struct.pack(f'<{len(values)}f', *values)
    
    elif type_ == 'float64':
        return struct.pack(f'<{len(values)}d', *values)
    
    elif type_ == 'string':
        # Encode strings as UTF-8 and concatenate with null terminators
        byte_array = bytearray()
        for value in values:
            byte_array.extend(value.encode('utf-8'))
            byte_array.append(0)  # Null terminator
        return bytes(byte_array)
    
    else:
        raise ValueError(f"Unsupported type: {type_}")

# Example usage:
# print(serialize_values('bool', [True, False, True, True]))
# print(serialize_values('int4', [5, 10]))
# print(serialize_values('float32', [1.23, 4.56]))
# print(serialize_values('string', ['hello', 'world']))
