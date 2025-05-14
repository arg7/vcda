import unittest
from opcode_parser import OpcodeParser, OPCODE_PARAMS  # Adjust import based on your file structure

class TestOpcodeParser(unittest.TestCase):
    def setUp(self):
        """Initialize the parser before each test."""
        self.parser = OpcodeParser()

    def test_valid_instruction_with_label_and_positional_arg(self):
        """Test parsing an instruction with a label and positional argument."""
        result = self.parser.parse("loop: JMP @loop")
        expected = {
            'label': 'loop',
            'opcode': 'JMP',
            'args': {'ofs': '@loop'}
        }
        self.assertEqual(result, expected)

    def test_valid_instruction_with_label_and_named_arg(self):
        """Test parsing an instruction with a label and named argument."""
        result = self.parser.parse("loop: JMP ofs=@loop")
        expected = {
            'label': 'loop',
            'opcode': 'JMP',
            'args': {'ofs': '@loop'}
        }
        self.assertEqual(result, expected)

    def test_valid_instruction_without_label(self):
        """Test parsing an instruction without a label."""
        result = self.parser.parse("INC r1, 5")
        expected = {
            'label': None,
            'opcode': 'INC',
            'args': {'reg': 'r1', 'val': '5'}
        }
        self.assertEqual(result, expected)

    def test_no_args_opcode(self):
        """Test parsing an opcode with no arguments."""
        result = self.parser.parse("NOP")
        expected = {
            'label': None,
            'opcode': 'NOP',
            'args': {}
        }
        self.assertEqual(result, expected)

    def test_optional_args_omitted(self):
        """Test parsing an opcode with optional arguments omitted."""
        result = self.parser.parse("RET")
        expected = {
            'label': None,
            'opcode': 'RET',
            'args': {}
        }
        self.assertEqual(result, expected)

    def test_optional_args_included(self):
        """Test parsing an opcode with optional arguments included."""
        result = self.parser.parse("RET 42")
        expected = {
            'label': None,
            'opcode': 'RET',
            'args': {'cnt': '42'}
        }
        self.assertEqual(result, expected)

    def test_complex_opcode_with_mixed_args(self):
        """Test parsing a complex opcode with mixed positional and named args."""
        result = self.parser.parse("ALU add, adt=8bit, r1, r2, dst=r3")
        expected = {
            'label': None,
            'opcode': 'ALU',
            'args': {
                'op': 'add',
                'adt': '8bit',
                'arg1': 'r1',
                'arg2': 'r2',
                'dst': 'r3'
            }
        }
        self.assertEqual(result, expected)

    def test_label_with_colon_only(self):
        """Test parsing a label with a colon but no args."""
        result = self.parser.parse("mylabel: PUSH r1")
        expected = {
            'label': 'mylabel',
            'opcode': 'PUSH',
            'args': {'reg': 'r1'}
        }
        self.assertEqual(result, expected)

    def test_whitespace_handling(self):
        """Test parsing with extra whitespace."""
        result = self.parser.parse("  loop:   JMP    @loop  ")
        expected = {
            'label': 'loop',
            'opcode': 'JMP',
            'args': {'ofs': '@loop'}
        }
        self.assertEqual(result, expected)

    def test_invalid_opcode(self):
        """Test parsing an invalid opcode."""
        with self.assertRaisesRegex(ValueError, r"Invalid opcode: XYZ"):
            self.parser.parse("XYZ @loop")

    def test_too_few_args(self):
        """Test parsing with too few arguments."""
        with self.assertRaisesRegex(ValueError, r"Missing required parameter 'reg' for opcode RS"):
            self.parser.parse("RS")

    def test_too_many_args(self):
        """Test parsing with too many arguments."""
        with self.assertRaisesRegex(ValueError, r"Too many positional arguments for opcode JMP"):
            self.parser.parse("JMP @loop, 1, 2")

    def test_invalid_param_name(self):
        """Test parsing with an invalid parameter name."""
        with self.assertRaisesRegex(ValueError, r"Invalid parameter name 'invalid' for opcode JMP"):
            self.parser.parse("JMP invalid=@loop")

    def test_duplicate_param(self):
        """Test parsing with duplicate parameters."""
        with self.assertRaisesRegex(ValueError, r"Duplicate parameter 'ofs' for opcode JMP"):
            self.parser.parse("JMP ofs=@loop, ofs=other")

    def test_invalid_instruction_format(self):
        """Test parsing an invalid instruction format."""
        with self.assertRaisesRegex(ValueError, r"Invalid instruction format"):
            self.parser.parse(":: JMP @loop")

    def test_empty_line(self):
        """Test parsing an empty or whitespace-only line."""
        with self.assertRaisesRegex(ValueError, r"Invalid instruction format"):
            self.parser.parse("   ")

    def test_mixed_positional_and_named_args(self):
        """Test parsing with a mix of positional and named arguments."""
        result = self.parser.parse("CALL func, bcs=1")
        expected = {
            'label': None,
            'opcode': 'CALL',
            'args': {'ofs': 'func', 'bcs': '1'}
        }
        self.assertEqual(result, expected)

    def test_all_opcodes_valid_args(self):
        """Test valid argument combinations for all opcodes."""
        test_cases = [
            ("NOP", {'label': None, 'opcode': 'NOP', 'args': {}}),
            ("RET 5", {'label': None, 'opcode': 'RET', 'args': {'cnt': '5'}}),
            ("IRET", {'label': None, 'opcode': 'IRET', 'args': {}}),
            ("INC r1, 10", {'label': None, 'opcode': 'INC', 'args': {'reg': 'r1', 'val': '10'}}),
            ("DEC r2", {'label': None, 'opcode': 'DEC', 'args': {'reg': 'r2'}}),
            ("NOT r3, 2", {'label': None, 'opcode': 'NOT', 'args': {'reg': 'r3', 'cnt': '2'}}),
            ("RS r4", {'label': None, 'opcode': 'RS', 'args': {'reg': 'r4'}}),
            ("NS ns1", {'label': None, 'opcode': 'NS', 'args': {'ns': 'ns1'}}),
            ("LI 42, r5", {'label': None, 'opcode': 'LI', 'args': {'val': '42', 'reg': 'r5'}}),
            ("JMP @target", {'label': None, 'opcode': 'JMP', 'args': {'ofs': '@target'}}),
            ("CALL func", {'label': None, 'opcode': 'CALL', 'args': {'ofs': 'func'}}),
            ("PUSH r6, 3", {'label': None, 'opcode': 'PUSH', 'args': {'reg': 'r6', 'cnt': '3'}}),
            ("POP r7, cnt=2, ofs=10", {'label': None, 'opcode': 'POP', 'args': {'reg': 'r7', 'cnt': '2', 'ofs': '10'}}),
            ("ALU add, u8, r1, r2, r3", {'label': None, 'opcode': 'ALU', 'args': {'op': 'add', 'adt': 'u8', 'arg1': 'r1', 'arg2': 'r2', 'dst': 'r3'}}),
            ("INT 7", {'label': None, 'opcode': 'INT', 'args': {'val': '7'}}),
            ("IN ch1, r8", {'label': None, 'opcode': 'IN', 'args': {'ch': 'ch1', 'reg': 'r8'}}),
            ("EXT ipg1", {'label': None, 'opcode': 'EXT', 'args': {'ipg': 'ipg1'}})
        ]
        for instruction, expected in test_cases:
            with self.subTest(instruction=instruction):
                result = self.parser.parse(instruction)
                self.assertEqual(result, expected)

    def test_positional_args_order(self):
        """Test that positional arguments are assigned to parameters in the correct order."""
        result = self.parser.parse("ALU add, u8, r2, r3, r4, 100")
        expected = {
            'label': None,
            'opcode': 'ALU',
            'args': {
                'op': 'add',
                'adt': 'u8',
                'arg1': 'r2',
                'arg2': 'r3',
                'dst': 'r4',
                'ofs': '100'
            }
        }
        self.assertEqual(result, expected)

    def test_JMP_at_arg(self):
        """Test that @label parsed correctly."""
        result = self.parser.parse("loop: JMP @loop, Always")
        expected = {
            'label': 'loop',
            'opcode': 'JMP',
            'args': {
                'bcs': 'Always',
                'ofs': '@loop'
            }
        }
        self.assertEqual(result, expected)

    def test_JMP_named_args(self):
        """Test JMP named args."""
        result = self.parser.parse("loop: JMP bcs=Always, ofs=@loop")
        expected = {
            'label': 'loop',
            'opcode': 'JMP',
            'args': {
                'bcs': 'Always',
                'ofs': '@loop'
            }
        }
        self.assertEqual(result, expected)

if __name__ == '__main__':
    unittest.main()