import unittest
from alloc_parser import serialize_values

class TestSerializeValues(unittest.TestCase):

    def test_bool(self):
        # Test serialization of boolean values
        self.assertEqual(serialize_values("bool", [True, True, False, True, True]), b'\x1b')  # 11011 in binary
        self.assertEqual(serialize_values("bool", [False, False, False, False]), b'\x00')  # 0000 in binary
        self.assertEqual(serialize_values("bool", [True] * 8), b'\xff')  # 11111111 in binary

    def test_int8(self):
        # Test serialization of 8-bit integers
        self.assertEqual(serialize_values("int8", [127, -128]), b'\x7f\x80')
        self.assertEqual(serialize_values("uint8", [255, 0]), b'\xff\x00')

    def test_int16(self):
        # Test serialization of 16-bit integers
        self.assertEqual(serialize_values("int16", [32767, -32768]), b'\xff\x7f\x00\x80')
        self.assertEqual(serialize_values("uint16", [65535, 0]), b'\xff\xff\x00\x00')

    def test_int32(self):
        # Test serialization of 32-bit integers
        self.assertEqual(serialize_values("int32", [2147483647, -2147483648]), b'\xff\xff\xff\x7f\x00\x00\x00\x80')
        self.assertEqual(serialize_values("uint32", [4294967295, 0]), b'\xff\xff\xff\xff\x00\x00\x00\x00')

    def test_float32(self):
        # Test serialization of 32-bit floats
        self.assertEqual(serialize_values("float32", [3.14, -2.71]), b'\xc3\xf5\x48\x40\x8f\xc2\x2d\xc0')

    def test_float64(self):
        # Test serialization of 64-bit floats
        self.assertEqual(serialize_values("float64", [3.14, -2.71]), b'\x1f\x85\xeb\x51\xb8\x1e\x09\x40\x8f\xc2\xf5\x28\x5c\x8f\x02\xc0')

    def test_char(self):
        # Test serialization of characters
        self.assertEqual(serialize_values("char", ['a', 'b', 'c']), b'abc')
        self.assertEqual(serialize_values("char", ['\x00', '\n']), b'\x00\n')

    def test_unsupported_type(self):
        # Test unsupported type
        with self.assertRaises(ValueError):
            serialize_values("unsupported_type", [1, 2, 3])
