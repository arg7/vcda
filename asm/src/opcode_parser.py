import re
from typing import Dict, List, Optional, Tuple

# Define valid opcodes and their parameters
OPCODE_PARAMS = {
    'NOP': [],
    'RET': [('cnt', True)],
    'IRET': [('cnt', True)],
    'INC': [('reg', True), ('val', True)],
    'DEC': [('reg', True), ('val', True)],
    'NOT': [('reg', True), ('cnt', True)],
    'RS': [('reg', False)],
    'NS': [('ns', False)],
    'LI': [('val', False), ('reg', True)],
    'JMP': [('ofs', False), ('bcs', True)],
    'CALL': [('ofs', False), ('bcs', True)],
    'PUSH': [('reg', False), ('cnt', True)],
    'POP': [('reg', False), ('cnt', True), ('ofs', True)],
    'ALU': [('op', False), ('adt', True), ('arg1', True), ('arg2', True), ('dst', True), ('ofs', True)],
    'INT': [('val', False)],
    'IN': [('ch', False), ('reg', True), ('adt', True), ('fmt', True)],
    'EXT': [('ipg', False)]
}

class OpcodeParser:
    
    def __init__(self):
        # Regex for parsing the instruction line
        self.line_pattern = re.compile(
            r'^\s*(?:(?P<label>[a-zA-Z_][a-zA-Z0-9_]*)\s*:)?\s*(?P<opcode>[a-zA-Z]+)\s*(?P<args>.*)?\s*$'
        )
        # Regex for splitting arguments (handles commas and named parameters)
        self.arg_pattern = re.compile(r'([^,=]+)(?:=([^,]*))?(?:,|$)')


    def parse(self, line: str) -> Dict:
        """
        Parse an assembly instruction line and return a structured dictionary.
        
        Args:
            line (str): The assembly instruction (e.g., "loop: JMP @loop" or "loop: JMP ofs=@loop")
        
        Returns:
            Dict: Structured output with label, opcode, and args
        """
        # Match the line against the pattern
        match = self.line_pattern.match(line.strip())
        if not match:
            raise ValueError(f"Invalid instruction format: {line}")

        label = match.group('label') or None
        opcode = match.group('opcode').upper()
        args_str = match.group('args') or ''

        # Validate opcode
        if opcode not in OPCODE_PARAMS:
            raise ValueError(f"Invalid opcode: {opcode}")

        # Parse arguments
        args = self._parse_args(args_str, opcode)
        
        return {
            'label': label,
            'opcode': opcode,
            'args': args
        }

    def _parse_args(self, args_str: str, opcode: str) -> Dict[str, str]:
        """
        Parse the argument string into a dictionary, handling both positional and named parameters.
        
        Args:
            args_str (str): The argument string (e.g., "@loop" or "ofs=@loop").
            opcode (str): The opcode to determine valid parameters.
        
        Returns:
            Dict[str, str]: Dictionary of parameter names to values.
        
        Raises:
            ValueError: If required parameters are missing or invalid arguments are provided.
        """
        # Get parameter specifications
        param_specs = OPCODE_PARAMS[opcode]
        param_names = [name for name, _ in param_specs]

        # Parse arguments
        matches = self.arg_pattern.finditer(args_str.strip())
        args_list = []
        for match in matches:
            key_or_value = match.group(1).strip()
            value = match.group(2)
            if value is not None:  # Named parameter
                args_list.append((key_or_value, value.strip()))
            else:  # Positional parameter
                args_list.append((None, key_or_value))

        result = {}
        named_params = set()

        # Process named parameters
        for key, value in args_list:
            if key is not None:
                if key not in param_names:
                    raise ValueError(f"Invalid parameter name '{key}' for opcode {opcode}")
                if key in result:
                    raise ValueError(f"Duplicate parameter '{key}' for opcode {opcode}")
                result[key] = value
                named_params.add(key)

        # Process positional parameters
        positional_values = [value for key, value in args_list if key is None]
        positional_index = 0

        for param_name, optional in param_specs:
            if param_name in named_params:
                continue
            if positional_index < len(positional_values):
                if param_name in result:
                    raise ValueError(f"Duplicate parameter '{param_name}' for opcode {opcode}")
                result[param_name] = positional_values[positional_index]
                positional_index += 1
            elif not optional:
                raise ValueError(f"Missing required parameter '{param_name}' for opcode {opcode}")

        # Check for excess positional arguments
        if positional_index < len(positional_values):
            raise ValueError(f"Too many positional arguments for opcode {opcode}")

        return result