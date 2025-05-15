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

        ### CALL Opcode Tests ###
    def test_call_small_ofs(self):
        """Test CALL with offset 2 (1-byte)."""
        parsed_instruction = {'opcode': 'CALL', 'args': {'ofs': '2'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x52')  # (0x5 << 4) | 2 = 0x52
        self.assertEqual(self.labels, {})

    def test_call_negative_ofs(self):
        """Test CALL with offset -1 (1-byte)."""
        parsed_instruction = {'opcode': 'CALL', 'args': {'ofs': '-1'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x5F')  # (0x5 << 4) | 0xF = 0x5F (-1 as 4-bit signed)
        self.assertEqual(self.labels, {})

    def test_call_with_bcs_small_ofs(self):
        """Test CALL with condition Z and offset 3 (2-byte)."""
        parsed_instruction = {'opcode': 'CALL', 'args': {'bcs': 'Z', 'ofs': '3'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC5\x13')  # 0xC5, (0x1 << 4) | 3 = 0x13
        self.assertEqual(self.labels, {})

    def test_call_missing_ofs(self):
        """Test CALL without ofs argument raises ValueError."""
        parsed_instruction = {'opcode': 'CALL', 'args': {}}
        with self.assertRaisesRegex(ValueError, "CALL requires 'ofs' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    def test_call_large_ofs(self):
        """Test CALL with offset > 7 and no bcs raises ValueError."""
        parsed_instruction = {'opcode': 'CALL', 'args': {'ofs': '10'}}
        with self.assertRaisesRegex(ValueError, "Offset too large for 1-byte CALL"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### PUSH Opcode Tests ###
    def test_push_small_reg(self):
        """Test PUSH with small register R.A (1-byte)."""
        parsed_instruction = {'opcode': 'PUSH', 'args': {'reg': 'R.A'}}  # R.A = 0x0
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x60')  # (0x6 << 4) | 0x0 = 0x60
        self.assertEqual(self.labels, {})

    def test_push_large_reg(self):
        """Test PUSH with large register R.IP (2-byte)."""
        parsed_instruction = {'opcode': 'PUSH', 'args': {'reg': 'R.IP'}}  # R.IP = 0xFF
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC6\xFF')  # 0xC6, 0xFF
        self.assertEqual(self.labels, {})

    def test_push_missing_reg(self):
        """Test PUSH without reg argument raises ValueError."""
        parsed_instruction = {'opcode': 'PUSH', 'args': {}}
        with self.assertRaisesRegex(ValueError, "PUSH requires 'reg' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    def test_push_reg_out_of_range(self):
        """Test PUSH with out-of-range register raises ValueError."""
        parsed_instruction = {'opcode': 'PUSH', 'args': {'reg': '256'}}  # Invalid register
        with self.assertRaisesRegex(ValueError, "Invalid argument: reg=256"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### POP Opcode Tests ###
    def test_pop_small_reg(self):
        """Test POP with small register R.A (1-byte)."""
        parsed_instruction = {'opcode': 'POP', 'args': {'reg': 'R.A'}}  # R.A = 0x0
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x70')  # (0x7 << 4) | 0x0 = 0x70
        self.assertEqual(self.labels, {})

    def test_pop_large_reg(self):
        """Test POP with large register R.IP (2-byte)."""
        parsed_instruction = {'opcode': 'POP', 'args': {'reg': 'R.IP'}}  # R.IP = 0xFF
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC7\xFF')  # 0xC7, 0xFF
        self.assertEqual(self.labels, {})

    def test_pop_missing_reg(self):
        """Test POP without reg argument raises ValueError."""
        parsed_instruction = {'opcode': 'POP', 'args': {}}
        with self.assertRaisesRegex(ValueError, "POP requires 'reg' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### NS Opcode Tests ###
    def test_ns_small_reg(self):
        """Test NS with small register R.A (1-byte)."""
        parsed_instruction = {'opcode': 'NS', 'args': {'reg': 'R.A'}}  # R.A = 0x0
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x20')  # (0x2 << 4) | 0x0 = 0x20
        self.assertEqual(self.labels, {})

    def test_ns_large_reg(self):
        """Test NS with large register R.IP (2-byte)."""
        parsed_instruction = {'opcode': 'NS', 'args': {'reg': 'R.IP'}}  # R.IP = 0xFF
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC2\xFF')  # 0xC2, 0xFF
        self.assertEqual(self.labels, {})

    def test_ns_missing_reg(self):
        """Test NS without reg argument raises ValueError."""
        parsed_instruction = {'opcode': 'NS', 'args': {}}
        with self.assertRaisesRegex(ValueError, "NS requires 'reg' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### ALU Opcode Tests ###
    def test_alu_no_adt(self):
        """Test ALU with operation ADD and no adt (1-byte)."""
        parsed_instruction = {'opcode': 'ALU', 'args': {'op': 'ADD'}}  # ADD = 0x0
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x80')  # (0x8 << 4) | 0x1 = 0x80
        self.assertEqual(self.labels, {})

    def test_alu_with_adt(self):
        """Test ALU with operation ADD and adt i32 (2-byte)."""
        parsed_instruction = {'opcode': 'ALU', 'args': {'op': 'ADD', 'adt': 'i32'}}  # ADD = 0x0, i32 = 0x5
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC8\x05')  # 0xC8, (0x1 << 4) | 0x3 = 0x13
        self.assertEqual(self.labels, {})

    def test_alu_missing_op(self):
        """Test ALU without op argument raises ValueError."""
        parsed_instruction = {'opcode': 'ALU', 'args': {}}
        with self.assertRaisesRegex(ValueError, "ALU requires 'op' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    def test_alu_large_op_no_adt(self):
        """Test ALU with op > 15 and no adt raises ValueError."""
        parsed_instruction = {'opcode': 'ALU', 'args': {'op': '16'}}  # Invalid op
        with self.assertRaisesRegex(ValueError, "Invalid argument: op=16"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### INT Opcode Tests ###
    def test_int_small_val(self):
        """Test INT with value 5 (1-byte)."""
        parsed_instruction = {'opcode': 'INT', 'args': {'val': '5'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\x95')  # (0x9 << 4) | 5 = 0x95
        self.assertEqual(self.labels, {})

    def test_int_large_val(self):
        """Test INT with value 16 (2-byte)."""
        parsed_instruction = {'opcode': 'INT', 'args': {'val': '16'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xC9\x10')  # 0xC9, 0x10
        self.assertEqual(self.labels, {})

    def test_int_missing_val(self):
        """Test INT without val argument raises ValueError."""
        parsed_instruction = {'opcode': 'INT', 'args': {}}
        with self.assertRaisesRegex(ValueError, "INT requires 'val' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    def test_int_val_out_of_range(self):
        """Test INT with value > 255 raises ValueError."""
        parsed_instruction = {'opcode': 'INT', 'args': {'val': '256'}}
        with self.assertRaisesRegex(ValueError, "Val 256 out of range"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### IN Opcode Tests ###
    def test_in_small_ch(self):
        """Test IN with channel 10 (1-byte)."""
        parsed_instruction = {'opcode': 'IN', 'args': {'ch': '10'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xAA')  # (0xA << 4) | 10 = 0xAA
        self.assertEqual(self.labels, {})

    def test_in_large_ch(self):
        """Test IN with channel 255 (2-byte)."""
        parsed_instruction = {'opcode': 'IN', 'args': {'ch': '255'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xCA\xFF')  # 0xCA, 0xFF
        self.assertEqual(self.labels, {})

    def test_in_missing_ch(self):
        """Test IN without ch argument raises ValueError."""
        parsed_instruction = {'opcode': 'IN', 'args': {}}
        with self.assertRaisesRegex(ValueError, "IN requires 'ch' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

    ### OUT Opcode Tests ###
    def test_out_small_ch(self):
        """Test OUT with channel 0 (1-byte)."""
        parsed_instruction = {'opcode': 'OUT', 'args': {'ch': '0'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xB0')  # (0xB << 4) | 0 = 0xB0
        self.assertEqual(self.labels, {})

    def test_out_large_ch(self):
        """Test OUT with channel 200 (2-byte)."""
        parsed_instruction = {'opcode': 'OUT', 'args': {'ch': '200'}}
        result = serialize_opcode(parsed_instruction, self.labels, 0)
        self.assertEqual(result, b'\xCB\xC8')  # 0xCB, 0xC8
        self.assertEqual(self.labels, {})

    def test_out_missing_ch(self):
        """Test OUT without ch argument raises ValueError."""
        parsed_instruction = {'opcode': 'OUT', 'args': {}}
        with self.assertRaisesRegex(ValueError, "OUT requires 'ch' argument"):
            serialize_opcode(parsed_instruction, self.labels, 0)

if __name__ == '__main__':
    unittest.main()