import unittest
from alloc_parser import encode_hex

class TestEncodeHex(unittest.TestCase):

    def test_basic(self):
        # Test encoding of bytes into hex string
        self.assertEqual(encode_hex(b'\x00\x01\x02\x03'), "00 01 02 03")
        self.assertEqual(encode_hex(b'\xff\xfe\xfd\xfc'), "ff fe fd fc")

    def test_empty_input(self):
        # Test encoding of empty bytes
        self.assertEqual(encode_hex(b''), "")

    def test_custom_bytes_per_line(self):
        # Test encoding with custom bytes_per_line
        self.assertEqual(encode_hex(b'\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f', 8),
                         "00 01 02 03 04 05 06 07\n08 09 0a 0b 0c 0d 0e 0f")

    def test_partial_line(self):
        # Test encoding with partial line
        self.assertEqual(encode_hex(b'\x00\x01\x02\x03\x04', 3), "00 01 02\n03 04")
    
