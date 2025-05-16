import re
from typing import Dict, List, Optional, Tuple
from alloc_parser import parse_array_declaration
from serialize import serialize_values
from ISA import OPCODE_PARAMS

class OpcodeParser:

    def __init__(self):
        # Regex for parsing the instruction line
        self.line_pattern = re.compile(r'^\s*(?:(?P<label>[a-zA-Z_][a-zA-Z0-9_]*)\s*:)?\s*(?P<opcode>[a-zA-Z]*)\s*(?P<args>.*)?\s*$')
        # Regex for splitting arguments (handles commas and named parameters)
        self.arg_pattern = re.compile(r'([^,=]+)(?:=([^,]*))?(?:,|$)')


    def parse_alloc(self, line):
        # Extract the array declaration part
        match = re.match(r'alloc\s+(.+)', line)
        if match:
            array_decl = match.group(1)
            # Parse the array declaration using the tested function
            t, l, v = parse_array_declaration(array_decl)
            # Serialize and encode the values using the tested functions
            return serialize_values(t, v)
        return None

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
        opcode = match.group('opcode').upper() if match.group('opcode') else None
        args_str = match.group('args') or None

        if opcode:
            # Validate opcode
            if opcode == 'ALLOC':
                # Alloc directives take up space based on the data
                data = self.parse_alloc(line)
                return {
                    'label': label,
                    'opcode': opcode,
                    'args': data,
                    'size': len(data)
                }
            elif opcode not in OPCODE_PARAMS:
                raise ValueError(f"Invalid opcode: {opcode}")

            # Parse arguments
            args = self._parse_args(args_str, opcode)
        else:
            opcode = None
            args = None
        
        if label or opcode: 
            return {
                'label': label,
                'opcode': opcode,
                'args': args,
                'size': 1 if opcode else 0 
            }
        else:
            return None


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

        if not args_str:
            return None
        
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