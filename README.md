# VD arch: Specification

Code Density Challenge, Very Dense Architecture with 8-bit instruction size, adaptable for any data width, 8, 16, 32, 64 bits and beyond.

## Definitions:

WS: defines CPU word size, ie 32;

HWS: defines half of WS;

## Registers

| Register | Description |
| --- | --- |
| R0 | First operand or pointer |
| R1 | Second operand or pointer |
| R2 | Result or pointer |
| R3 | High part of result (MUL) or first operand (DIV) |
| R4 | SIMD\_CTRL: VL[HWS] | Stride\_R2[HWS] |
| R5 | SIMD\_STRIDE: Stride\_R0[HWS] | Stride\_R1[HWS] |
| R6–R8 | Free registers |
| R9 | ITBP: Interrupt Table Base Pointer |
| R10 | ICTRL: Pending[HWS] | Masked[HWS] |
| R11 | FLAGS: Status and control flags |
| R12 | JMP\_Stride: Jump stride |
| R13 | BP: Base Pointer |
| R14 | SP: Stack Pointer |
| R15 | IP: Instruction Pointer |



Note: 
> SIMD\_CTRL register defines two fields, VL (Vector Length) unsigned int of HWS bit size and Stride\_R2 as signed int of same size, which set the behavior of storing result of ALU operation.
> SIMD\_STRIDE register defines two fields, Stride\_R0 and Stride\_R1 as signed int of HWS bit size, which sets the behavior of input operands in ALU operation.

## FLAGS (R11)

| Bit(s) | Description |
| --- | --- |
| 0 | Carry Flag (C) |
| 1 | Zero Flag (Z) |
| 2 | Sign Flag (S) |
| 3 | Overflow Flag (V) |
| 4 | Parity Flag (P) |
| 5 | LI Execution Flag (L) |
| 6 | SIMD Execution Flag (S) |
| 7–10 | BCS, Branch Condition Selector |
| 11–14 | RS, Register Selector |
| 15–18 | ADT, ALU Data Type Selector |



## ALU Data Type Selector (4 bits)

| Value | Operand Type | Description |
| --- | --- | --- |
| 0x0 | uint8 | 8-bit unsigned integer |
| 0x1 | int8 | 8-bit signed integer |
| 0x2 | uint16 | 16-bit unsigned integer |
| 0x3 | int16 | 16-bit signed integer |
| 0x4 | uint32 | 32-bit unsigned integer |
| 0x5 | int32 | 32-bit signed integer |
| 0x6 | uint64 | 64-bit unsigned integer |
| 0x7 | int64 | 64-bit signed integer |
| 0x8 | float16 | 16-bit floating-point (half-precision) |
| 0x9 | float32 | 32-bit floating-point (single-precision) |
| 0xA | float64 | 64-bit floating-point (double-precision) |
| 0xB | BOOL | 1-bit Boolean type |
| 0xC | INT4 | 4-bit int type |
| 0xD | FP4 | 4-bit floating-point |
| 0xE | FP8 | 8-bit floating-point |
| 0xF | reserved |  |

## Branch Condition Selector (4-bit)

| Value | Condition |
| --- | --- |
| 0x0 | Always |
| 0x1 | Zero (JZ) |
| 0x2 | Not Zero (JNZ) |
| 0x3 | Greater (JG) |
| 0x4 | Greater or Equal (JGE) |
| 0x5 | Less (JL) |
| 0x6 | Less or Equal (JLE) |
| 0x7 | Carry (JC) |
| 0x8 | Not Carry (JNC) |
| 0x9 | Sign (JS) |
| 0xA | Not Sign (JNS) |
| 0xB | Overflow (JO) |
| 0xC | Not Overflow (JNO) |
| 0xD | Parity Even (JPE) |
| 0xE | Parity Odd (JPO) |
| 0xF | reserved |

## ALU Operation Mode Selector (4-bit):

| Value | Operation | Description |
| --- | --- | --- |
| 0x0 | ADD | Addition |
| 0x1 | SUB | Subtraction |
| 0x2 | AND | Bitwise AND |
| 0x3 | OR | Bitwise OR |
| 0x4 | XOR | Bitwise Exclusive OR |
| 0x5 | SHL | Shift Left |
| 0x6 | SHR | Shift Right Logical |
| 0x7 | SAR | Shift Arithmetic Right (preserves sign) |
| 0x8 | NOT | Bitwise NOT (invert all bits) |
| 0x9 | CMP | Compare (sets flags based on comparison) |
| 0xA | INC | Increment |
| 0xB | DEC | Decrement |
| 0xC | MUL | Multiplication |
| 0xD | DIV | Division |
| 0xE | STRSTR | Search for substring |
| 0xF | Reserved |  |



## Instruction Set

Every instruction is one byte length. First 4 bits for opcode and last 4 bit for operand.

| Opcode 4-bit | Immed 4-bit | Name | Description |
| --- | --- | --- | --- |
| 0x0 | 0x0 | NOP | No operation |
| 0x0 | 0x1 | RET | Return from subroutine |
| 0x0 | 0x2 | IRET | Return from interrupt |
| 0x1 | reg | RS | Set FLAGS.RS |
| 0x2 | val | LI | Load unsigned immediate.If FLAGS.L==0 {reg_file[FLAGS.RS] = val; FLAGS.L=1;} Else {reg_file[FLAGS.RS]=(reg_file[FLAGS.RS] << 4)|val;} Any other instruction set FLAGS.L=0 |
| 0x3 | val | LIS | Load Immediate Signed, same logic as above, but extends sign bit on first assigment. |
| 0x4 | type | ADT | Sets ALU Data Type (FLAGS.ADT) |
| 0x5 | op | ALU | If SIMD\_CTRL.VL==0, performs R0 <op> R1 = (R3|R2); If SIMD\_CTRL.VL==1, performs [R0] <op> [R1] = [R2]; If SIMD\_CTRL.VL>1, performs [R0] <op> [R1] = [R2], increasing pointers by their Stride\_R[0|1|2] for each vector element. If Stride\_R[0|1] is 0, it means that register works as constant. If Stride\_R2 is 0, it means that register R2 works as accumulator. |
| 0x6 | cond | CS | Set condition to FLAGS.BCS |
| 0x7 | offset | JMP | Conditional (FLAGS.BCS) relative jump, effective address is calculated by IP = IP + JMP\_Stride*offset. Offset is 4-bit signed int; |
| 0x8 | offset | CALL | Conditional (FLAGS.BCS) relative call, same as above. |
| 0x9 | reg | PUSH | Push register onto the stack |
| 0xA | reg | POP | Pop value from the stack into register |
| 0xB | intn | INT | Trigger software interrupt <intn> |



Note:

>LI loads to register identified by FLAGS.RS only 4 bits of data at the time; LI can be chained to load arbitrary length constants to the register. Instruction decoder can detect RS, LI sequences and optimize it by assigning to FLAGS.RS register value combined from LI sequence in one cycle.
>ALU STRSTR, SIMD instruction searches for substring, pointed by [R0] in string pointed by [R1]. VL is set to string length, Stride\_R0 set to the length of substring and Stride\_R1 is not used. STRSTR returns in R2 value of -1, in case if match not found, or positive index of matched subsring.
>To find next occurrence, just repeat ALU STRSTR instruction, it will return next occurrence, if any.
>It works by setting R2 to -1 before first run.
>
>PUSH/POP can be used to perform register data move, like PUSH R2; POP IP, which assigns R2 to IP. Instruction decoder can optimize it out and assign values directly and avoid two memory operations.
>
>JMP and CALL have short and long forms:
>
>Short form: JMP\_Stride must be set to known small value (1, 2 or 4) and effective relative address is calculated by multiplying JMP\_Stride\*offset, so maximum range could from 4\*-8 to 4\*7, so -32 to 28 bytes; If bigger JMP\_Stride is used, better range is possible, but code density will suffer. Gaps could be filled by NOP.
>If condition and JMP\_Stride are the same, JMP/CALL instruction can be just one byte.
>
>Long form: JMP\_Stride is set to effective relative address, and offset is set to 1. But JMP\_Stride will need to be adjusted for every JMP/CALL.
>
>MOV emulation: by using SMD instruction all kinds of memory manipulation can be done, for example:
>
>memcopy: set R0 to address of block to copy; set R1 to zero; set R2 as pointer to memory block to receive copy; set data type to uint8 (it is possible to use unit64, but wary of memory alignments); set VL to source block length in data size units; set Stride\_R0 and Stride\_R1 to 1 and Stride\_R1 to zero. Launch ALU OR instruction.
>
>memcopy in reverse order: same as above, but set R2 pointer to the end of memory block and Stride\_R2 to -1;
>
>memset: same as memcopy, but set R0 and Stride\_R0 to zero.

## Assembly Sugar

### LI <reg>, <val>
Macro to set any register with arbitrary length immediate value.
Translates into a sequence of RS and LI instructions to load a multi-byte value into a register.
Example:
```
 LI R0, 0x12345678 
```
Translates to:
```
RS R0; LI 1; LI 2; LI 3; LI 4; LI 5; LI 6; LI 7; LI 8 
```
### JS <JMP\_stride>
Macro to set JMP stride value into R12 register. Same as LI R12, <JMP\_stride>

### JMP <offset|\@label> [if <contition>]

If a condition is specified, the assembler translates the instruction into:
```
 CS <condition>; JS <offset>; JMP 0x1 
```
or, for short jumps:
```
CS <condition>; JMP <offset>
```
If no condition is specified, the assembler uses CS Always, for unconditional JMP. 
```
CALL <offset|\@label> [if <contition>]
```
Same as JMP.


### SIMD

To simplyfy SIMD operations, we define SIMD macro, with following syntax:
```
 SIMD type[vl] [R2|stride] = ([R0|stride] <op> [R1|stride])
```
Where:
- type: data type, uint8 ecc;
- vl: vector length, uint bit length of HWS;
- op: ALU operation, see ALU Operation Mode Selector table;
- stride: distance between operands in memory,  uint bit length of HWS;

Note: when registers are not surrounded by [], it means that is has stride=0, and acts as a constant or accumulator. In case of accumulator, R3 is used to keep upper part of result, in case of R2 overflow.

Note: for operation both arithmetic symbols (+,-,\*,/, <<, >>) and mnemonics (ADD, SUB, MUL, DIV, SHL, SHR and rest from ALU Operation Mode Selector table) can be used.

Example: to sum 8 bytes referenced by R0 and R1 pointers, with increment by one and save results in memory referenced by R2, decremented by 4 (reverse order):
```
 SIMD uint8[8] ([R2|-4] = [R0] + 32)
```
Note: when stride is not specified, it defaults to data type size, ie sizeof(uint8) = 1;

Which translates to:
```
LI SIMD\_CTRL, 8 << HWS | -4; // vl = 8, Stride\_r2 = -4
LI SIMD\_STRIDE, 1; Stride\_r0 = 1, Stride\_r1 = 0
LI R1, 32; // load constant in R1
ADT uint8;
ALU ADD; // mapped from ‘+’ symbol
```

In case of STRSTR operation, SIMD instruction searches for substring, pointed by [R0] in string pointed by [R1]. VL is set to string length, Stride\_R0 to the length of substring and Stride\_R1 is not used. STRSTR returns in R2 value of -1, in case if match not found, or positive index of matched subsring. Stride_R2 must be 1 for forward search and -1 for reverse search. ADT uint8 set data type to char, but in general any data type can be used.

To find next occurrence, just repeat ALU STRSTR instruction, it will return next occurrence, if any.

Example: to find index of substring “the” in string “Here they come”
```
SIMD (R2 = StrStr(“Here they come”, “the”))
```
Which translates to:
```
s0001: db “Here they come”
s0002: db “the”
LI SIMD_CTRL, sizeof(s0001) << HWS; // vl = string size, stride_r2 = 0
LI SIMD_STRIDE, sizeof(s0002); stride_r0 = size of substring, stride_r1 = 0
LI R1, @s0001; // load string address in R1
LI R0, @s0002; // load substring address in R0
ADT uint8;     // set type to char
ALU STRSTR;    // perform search
```
Example: realize memset( buf, sizeof(buf), 0x55) with SIMD macro:

>SIMD uint8[sizeof(buf)] (@buf = 0x55);

Which translates to:
```
LI SIMD_CTRL, sizeof(buf) << HWS | 1; // vl = buf size, stride_r2 = 1
LI SIMD_STRIDE, 0; stride_r0 and stride\_r1 = 0
LI R2, @buf; // load str address in R2
LI R0, 0x55; // R0 = 0x55
LI R1, 0; // R1 = 0
ADT uint8;
ALU OR
```

### Assembly Notes

### Unconditional Rule:

If no condition is specified for JMP or CALL, the assembler automatically inserts CS 0x0 (unconditional).

### SIMD Mode:

When VL > 1, the architecture operates in SIMD mode:

R0, R1, and R2 are treated as memory pointers.
After each operation, the pointers are incremented by their respective strides.
If a stride is 0, the corresponding register is treated as a constant or accumulator.

### Interrupt Handling:

ITBP (R9) points to the base of the interrupt table.
ICTRL (R10) tracks pending and masked interrupts.

INT triggers a software interrupt.
IRET returns from an interrupt.



## Example Program

Here’s an example program that demonstrates the use of assembly sugar:
```
  // SIMD memory tests
  JS 1;
  JMP @start;
  
  buf: db[32];
  
start:
  // zero buf memory
  SIMD uint8[sizeof(buf)] (@buf = 0);
  //check if buf has all zeros, by summing all bytes
  SIMD uint8[sizeof(buf)] (R2 = @buf + 0);
  PUSH R2
  POP R0
  LI R1, 0
  SIMD uint32 (R0 CMP R1)
  JMP @ok if Equal
  
  fail:
  // test failed
  JMP @fail;
  
  ok:
  // everything is ok
  JMP @ok
  ```
  End
  
