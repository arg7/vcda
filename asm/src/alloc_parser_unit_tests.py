import unittest
from alloc_parser import parse_array_declaration

class TestParser(unittest.TestCase):

    def test_basic_types(self):
        self.assertEqual(parse_array_declaration("int8 = 0"), ("int8", 1, [0]))
        self.assertEqual(parse_array_declaration("uint16[] = {1,2,3,4}"), ("uint16", 4, [1, 2, 3, 4]))
        self.assertEqual(parse_array_declaration("float32[2] = {3.14, 2.71}"), ("float32", 2, [3.14, 2.71]))

    def test_constants(self):
        self.assertEqual(parse_array_declaration("int64 = 0xABCD"), ("int64", 1, [43981]))
        self.assertEqual(parse_array_declaration("uint8 = 0b1010"), ("uint8", 1, [10]))
        self.assertEqual(parse_array_declaration("mytype = {0x1, 0x2, 0b11}"), ("mytype", 3, [1, 2, 3]))

    def test_array_size(self):
        self.assertEqual(parse_array_declaration("int[5]"), ("int", 5, [0, 0, 0, 0, 0]))  # Corrected
        self.assertEqual(parse_array_declaration("int[5] = {1,2}"), ("int", 5, [1, 2, 0, 0, 0]))
        self.assertEqual(parse_array_declaration("int[]"), ("int", 1, [0])) # Or handle as you wish
        self.assertEqual(parse_array_declaration("int"), ("int", 1, [0])) # Or [] if you define as empty array


    def test_no_values(self):
        self.assertEqual(parse_array_declaration("bool = 1"), ("bool", 1, [1]))

    def test_char_literal(self):
        self.assertEqual(parse_array_declaration("char = 'a'"), ("char", 1, ['a']))
        self.assertEqual(parse_array_declaration("char[] = {'a', 'b', 'c'}"), ("char", 3, ['a', 'b', 'c'])) 

    def test_escape_chars(self):
        self.assertEqual(parse_array_declaration("char[1] = '\\0'"), ("char", 1, ['\x00']))
        self.assertEqual(parse_array_declaration("char[1] = '\\n'"), ("char", 1, ['\n']))

    def test_string_literal(self):
        self.assertEqual(parse_array_declaration('char[] = "string"'), ('char', 7, ['s', 't', 'r', 'i', 'n', 'g', '\x00']))
        self.assertEqual(parse_array_declaration('char[5] = "abc"'), ('char', 5, ['a', 'b', 'c', '\0', '\0'])) 

    def test_no_values_error(self):
        self.assertIsNone(parse_array_declaration("bool ="))  # Error case
        self.assertEqual(parse_array_declaration("bool[5] = {1}"), ("bool", 5, [1,0,0,0,0])) # correct case

