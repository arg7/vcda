import unittest
from alloc_parser import serialize_values
from alloc_parser import encode_hex

class TestSerializeValues(unittest.TestCase):

    def test_bool(self):
        # Test serialization of boolean values
        self.assertEqual(serialize_values("u1", [True, True, False, True]), b'\xb0')  # 11011 in binary
        self.assertEqual(serialize_values("u1", [False, False, False, False]), b'\x00')  # 0000 in binary
        self.assertEqual(serialize_values("u1", [True] * 8), b'\xff')  # 11111111 in binary

    def test_int8(self):
        # Test serialization of 8-bit integers
        self.assertEqual(serialize_values("i8", [127, -128]), b'\x7f\x80')
        self.assertEqual(serialize_values("u8", [255, 0]), b'\xff\x00')

    def test_int16(self):
        # Test serialization of 16-bit integers
        self.assertEqual(serialize_values("i16", [32767, -32768]), b'\xff\x7f\x00\x80')
        self.assertEqual(serialize_values("u16", [65535, 0]), b'\xff\xff\x00\x00')

    def test_int32(self):
        # Test serialization of 32-bit integers
        self.assertEqual(serialize_values("i32", [2147483647, -2147483648]), b'\xff\xff\xff\x7f\x00\x00\x00\x80')
        self.assertEqual(serialize_values("u32", [4294967295, 0]), b'\xff\xff\xff\xff\x00\x00\x00\x00')

    def test_float32(self):
        # Test serialization of 32-bit floats
        self.assertEqual(encode_hex(serialize_values("f32", [3.14, -2.71])), 'c3 f5 48 40 a4 70 2d c0')

    def test_float64(self):
        # Test serialization of 64-bit floats
        self.assertEqual(encode_hex(serialize_values("f64", [3.14, -2.71])), '1f 85 eb 51 b8 1e 09 40 ae 47 e1 7a 14 ae 05 c0')

    def test_char(self):
        # Test serialization of characters
        self.assertEqual(serialize_values("char", ['a', 'b', 'c']), b'abc')
        self.assertEqual(serialize_values("char", ['\x00', '\n']), b'\x00\n')

    def test_unsupported_type(self):
        # Test unsupported type
        with self.assertRaises(ValueError):
            serialize_values("unsupported_type", [1, 2, 3])
