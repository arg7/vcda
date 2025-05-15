import unittest
from asm import serialize_opcode

class TestSerializeOpcode(unittest.TestCase):
    def setUp(self):
        """Reset labels dictionary before each test."""
        self.labels = {}

    ### NOP_EXT Opcodes Tests ###
    def test_nop_no_args(self):
        """Test NOP with no arguments (1-byte)."""
        parsed_instruction = {'opcode': 'NOP'}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x00')
        self.assertEqual(self.labels, {})

    def test_ret_no_args(self):
        """Test RET with no arguments (1-byte)."""
        parsed_instruction = {'opcode': 'RET'}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x01')
        self.assertEqual(self.labels, {})

    def test_ret_with_cnt(self):
        """Test RET with cnt=5 (2-byte)."""
        parsed_instruction = {'opcode': 'RET', 'args': {'cnt': '5'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC0\x15')  # 0xC0, (0x1 << 4) | 5 = 0x15
        self.assertEqual(self.labels, {})

    ### Regular Opcodes Tests ###
    def test_rs_small_reg(self):
        """Test RS with small register R.A (1-byte)."""
        parsed_instruction = {'opcode': 'RS', 'args': {'reg': 'R.A'}}  # R.A = 0x0
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x10')  # (0x1 << 4) | 0x0 = 0x10
        self.assertEqual(self.labels, {})

    def test_rs_large_reg(self):
        """Test RS with large register R.IP (2-byte)."""
        parsed_instruction = {'opcode': 'RS', 'args': {'reg': 'R.IP'}}  # R.IP = 0xFF
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC1\xFF')  # 0xC1, 0xFF
        self.assertEqual(self.labels, {})

    def test_li_no_reg_small_val(self):
        """Test LI with value 5 and no register (1-byte)."""
        parsed_instruction = {'opcode': 'LI', 'args': {'val': '5'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x35')  # (0x3 << 4) | 5 = 0x35
        self.assertEqual(self.labels, {})

    def test_li_with_reg_small_val(self):
        """Test LI with register R.A and value 3 (2-byte)."""
        parsed_instruction = {'opcode': 'LI', 'args': {'reg': 'R.A', 'val': '3'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC3\x03')  # 0xC3, (0x0 << 4) | 3 = 0x03
        self.assertEqual(self.labels, {})

    def test_jmp_small_ofs(self):
        """Test JMP with offset 2 (1-byte)."""
        parsed_instruction = {'opcode': 'JMP', 'args': {'ofs': '2'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x42')  # (0x4 << 4) | 2 = 0x42
        self.assertEqual(self.labels, {})

    def test_jmp_negative_ofs(self):
        """Test JMP with offset -1 (1-byte)."""
        parsed_instruction = {'opcode': 'JMP', 'args': {'ofs': '-1'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x4F')  # (0x4 << 4) | 0xF = 0x4F (-1 as 4-bit signed)
        self.assertEqual(self.labels, {})

    def test_jmp_with_bcs_small_ofs(self):
        """Test JMP with condition Z and offset 3 (2-byte)."""
        parsed_instruction = {'opcode': 'JMP', 'args': {'bcs': 'Z', 'ofs': '3'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC4\x13')  # 0xC4, (0x1 << 4) | 3 = 0x13
        self.assertEqual(self.labels, {})

    ### Label Handling Tests ###
    def test_label_definition(self):
        """Test defining a new label with NOP."""
        parsed_instruction = {'label': 'start', 'opcode': 'NOP'}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x00')
        self.assertEqual(self.labels, {'start': 0})

    def test_duplicate_label(self):
        """Test defining a duplicate label raises ValueError."""
        parsed_instruction = {'label': 'start', 'opcode': 'NOP'}
        self.labels = {'start': 10}
        with self.assertRaisesRegex(ValueError, "Duplicate label: start"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    def test_jmp_with_label(self):
        """Test JMP with a defined label."""
        parsed_instruction = {'opcode': 'JMP', 'args': {'ofs': '@loop'}}
        self.labels = {'loop': 10}
        result = serialize_opcode(parsed_instruction, self.labels, 5)
        self.assertEqual(result, b'\x44')  # ofs = 10 - 5 - 1 = 4, so 0x44
        self.assertEqual(self.labels, {'loop': 10})

    def test_undefined_label(self):
        """Test using an undefined label raises ValueError."""
        parsed_instruction = {'opcode': 'JMP', 'args': {'ofs': '@unknown'}}
        with self.assertRaisesRegex(ValueError, "Undefined label: unknown"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### Error Cases Tests ###
    def test_rs_missing_reg(self):
        """Test RS without reg argument raises ValueError."""
        parsed_instruction = {'opcode': 'RS'}
        with self.assertRaisesRegex(ValueError, "RS requires 'reg' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    def test_li_large_val_no_reg(self):
        """Test LI with value > 15 and no reg raises ValueError."""
        parsed_instruction = {'opcode': 'LI', 'args': {'val': '16'}}
        with self.assertRaisesRegex(ValueError, "Value too large for 1-byte LI without reg"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    def test_jmp_large_ofs(self):
        """Test JMP with offset > 7 and no bcs raises ValueError."""
        parsed_instruction = {'opcode': 'JMP', 'args': {'ofs': '10'}}
        with self.assertRaisesRegex(ValueError, "Offset too large for 1-byte JMP"):
            serialize_opcode(parsed_instruction, self.labels, 0)

if __name__ == '__main__':
    unittest.main()