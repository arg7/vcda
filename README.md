# VD arch: Specification

Code Density Challenge, Very Dense Architecture with 8-bit instruction size, adaptable for any word width, 8, 16, 32, 64 bits and beyond.

## Definitions:

WS: defines CPU word size, ie 32;

HWS: defines half of WS;

## Registers

| Register | Description |
| --- | --- |
| R0–R15 | general purpose registers, one nibble address |
| R16–R247 | general purpose registers |
| R248 | ALU VR\_CTRL: VL[HWS] | ST\_RDST[HWS] |
| R249 | ALU VR\_STRIDE: ST\_RRS[HWS] ST\_RSRC[HWS] |
| R250 | FL: Status and control flags |
| R251 | JMP\_Stride: Jump stride, defaults to 1 |
| R252 | BP: Base Pointer |
| R253 | SP: Stack Pointer |
| R254 | IP: Instruction Pointer |
| R255 | FLAG register |



Note: 
> ALU VR\_CTRL register defines two fields, VL (Vector Length) unsigned int of HWS bit size and ST\_RDST as signed int of same size, which set the behavior of storing result of ALU operation.
> ALU VR\_STRIDE register defines two fields, ST\_RRS and ST\_RSRC as signed int of HWS bit size, which sets the behavior of input operands in ALU operation.
>VL is in ALU Data Type units, one byte for ADT.u8, 4 for ADT.u32.
>These registers can be used as general purpose, if ALU is not used. Using INC, DEC, NOT and CMP is ok.

## FLAGS (R255)

| Nibble | Description |
| --- | --- |
| 0 | NS, Nibble Selector, low, default 0 |
| 1 | NS, Nibble Selector, high, default 0 |
| 2 | RS, Register Selector, low, default 0 |
| 3 | RS, Register Selector, high, default 0|
| 4 | SRC Reg, low, default 1 |
| 5 | SRC Reg, high, default 0 |
| 6 | DST Reg, low, default 2 |
| 7 | DST Reg, high, default 0 |

| 8 | BCS, Branch Condition Selector, low |
| 9 | BCS, Branch Condition Selector, high |

| 10 | ADT, ALU Data Type Selector, low, default 0 |
| 11 | ADT, ALU Data Type Selector, high, default 0 |

| 12 | Carry Flag (C), Interrupt (I) |

Note:
This register is 64 bit on all WS, require special handling in PUSH/POP

## ALU Data Type Selector (4 bits)

| Value | Operand Type | Description |
| --- | --- | --- |
| 0x0 | u8 | 8-bit unsigned integer |
| 0x1 | i8 | 8-bit signed integer |
| 0x2 | u16 | 16-bit unsigned integer |
| 0x3 | i16 | 16-bit signed integer |
| 0x4 | u32 | 32-bit unsigned integer |
| 0x5 | i32 | 32-bit signed integer |
| 0x6 | u64 | 64-bit unsigned integer |
| 0x7 | i64 | 64-bit signed integer |
| 0x8 | f16 | 16-bit floating-point (half-precision) |
| 0x9 | f32 | 32-bit floating-point (single-precision) |
| 0xA | f64 | 64-bit floating-point (double-precision) |
| 0xB | u1 | 1-bit boolean type |
| 0xC | i4 | 4-bit int type |
| 0xD | fp4 | 4-bit floating-point |
| 0xE | fp8 | 8-bit floating-point |
| 0xF | reserved |  |

## Branch Condition Selector (4-bit)

| Value | Condition |
| --- | --- |
| 0x0 | Always |
| 0x1 | Zero (Z) |
| 0x2 | Not\_Zero (NZ) |
| 0x3 | Greater (G) |
| 0x4 | Greater\_Or\_Equal (GE) |
| 0x5 | Less (L) |
| 0x6 | Less\_Or\_Equal (LE) |
| 0x7 | Carry (C) |
| 0x8 | Not\_Carry (NC) |
| 0x9 | Sign (S) |
| 0xA | Not\_Sign (NS) |
| 0xB | Overflow (O) |
| 0xC | Not\_Overflow (NO) |
| 0xD | Parity\_Even (PE) |
| 0xE | Parity\_Odd (PO) |
| 0xF | Interrupt (I)|

## ALU Operation Mode Selector (4-bit):

| Value | Operation | Description |
| --- | --- | --- |
| 0x0 | ADD | Addition |
| 0x1 | ADDC | Addition with carry in/out|
| 0x2 | SUB | Subtraction |
| 0x3 | SUBC | Subtraction with borrow in/out|
| 0x4 | AND | Bitwise AND |
| 0x5 | OR | Bitwise OR |
| 0x6 | XOR | Bitwise Exclusive OR |
| 0x7 | SHL | Shift Left |
| 0x8 | SHR | Shift Right Logical |
| 0x9 | MUL | Multiplication |
| 0xA | DIV | Division |
| 0xB | LOOKUP | Lookup slice in vector|
| 0xC | LOAD | Loads nimble, byte or word |
| 0xD | STORE | Saves nimble, byte or word |
| 0xE-0xF | Reserved |  |

Note:

LOAD fetches data from memory, where R[N.SRC] is a base pointer and R[N.RS] is index and saves it to R[N.DST], if ST_RDST is 0, otherwise to memory pointed by R[N.DST]

SAVE same as LOAD by in reverse direction.

Size of data is determinied by FMT instruction, in case of NIBBLE, NS instruction can specify which nibble in R[N.DST] will be affected.

## Input/Output Map
| Value | Operation | Description |
| --- | --- | --- |
| 0x0 | STDIO | stdin/out channel  |
| 0x1 | STDERR | stderr channel  |

## Instruction Set

Every instruction is one byte length. First 4 bits for opcode and last 4 bit for operand.

| Opcode 4-bit | Immed 4-bit | Name | Description |
| --- | --- | --- | --- |
| 0x0 | 0x0 | NOP | No operation |
| 0x0 | 0x1 | RET | Return from subroutine |
| 0x0 | 0x2 | IRET | Return from interrupt |
| 0x0 | 0x3 | SETC | Set FL.C |
| 0x0 | 0x4 | CLSC | Zero FL.C |
| 0x0 | 0x5 | INC | Increment R[N.RS]|
| 0x0 | 0x6 | DEC | Decrement R[N.RS]|
| 0x0 | 0x7 | NOT | Bitwise NOT R[N.RS] |
| 0x1 | reg | RS | Set N.RS |
| 0x2 | reg | NS | Set N.NS |
| 0x3 | val | LI | Load unsigned immediate to register[N.RS] nibble[N.NS++] |
| 0x4 | val | LIS | Load Immediate Signed, same logic as above, but extends sign bit on first assigment. |
| 0x5 | op | ALU | Performs ALU operation, see table "ALU Operation Mode Selector" |
| 0x6 | offset | JMP | Conditional (N.BCS) relative jump, effective address is calculated by IP = IP + JMP\_Stride*offset. Offset is 4-bit signed int; |
| 0x7 | offset | CALL | Conditional (N.BCS) relative call, same as above. |
| 0x8 | reg | PUSH | Push register onto the stack |
| 0x9 | reg | POP | Pop value from the stack into register |
| 0xa | intn | INT | Trigger software interrupt <intn> |
| 0xb | val | IN | Read byte to R[N.RS] from i/o channel <val>, FL.Z is 0, if successfull |
| 0xc | val | OUT | Write byte from R[N.RS] to i/o channel <val>, FL.Z is 0, if successfull|


## Note:

### RS
**RS** instruction resets **N.NS** to 0.

### LI/LIS
**LI** loads immediate u4 to nibble **N.NS** of register **N.RS**; **LI** can be chained to load arbitrary constants. If previous instruction was **LI**, it assigns **immediate u4** to the next nibble in the register. 
In case of **signed LIS**, it also extend i4 sign to the all the upper nibbles of register.

*Optimization:* instruction decoder can detect **RS**, **NS** and **LI**/**LIS** sequences and optimize by assigning to **N.RS** register value combined from **LI**/**LIS** sequence in one cycle.

### ALU
- **Scalar register**, If **VR\_CTRL.VL==0**, performs 
```
R[N.DST] = R[N.RS] <op> R[N.SRC]
```
- **Scalar memory**, If **VR\_CTRL.VL==1**, performs 
```
M[R[N.DST]] = M[R[N.RS]] <op> M[R[N.SRC]]
```
- **Vector memory**, If **VR\_CTRL.VL > 1**, performs
```
M[R[N.DST]] = M[R[N.RS]] <op> M[R[N.SRC]]
```
increasing pointers by their repective stride for each vector element. If **stride** is 0, it means that register works as constant. If **ST\_RDST** is 0, it means that register **R[N.DST]** works as accumulator.

### ALU LOOKUP
**ALU LOOKUP**, vector instruction searches for slice **M[R[N.RS]]** in a vector **M[R[N.SRC]]**. **VL** is set to vector length, **ST\_RRS** set to the length of slice and **ST\_RSRC** is not used. **LOOKUP** returns in **R[N.DST]** the index of found slice and sets FL.C to true, otherwise sets FL.C to false.

To find next occurrence, increase **R[N.DST]** and repeat **ALU LOOKUP** instruction, it will return next occurrence, if any.

### memcpy, memset emulation
by using **ALU vector mode**, many memory manipulation triks can be done, for example:

- **memcopy**: set **R[N.RS]** to address of block to copy; set **R[N.SRC]** to zero; set **R[N.DST]** as pointer to memory block to receive copy; set data type to **uint8** (it is possible to use **unit64**, but wary of memory alignments); set **VL** to source block length in data size units; set **ST\_RRS** and **ST\_RSRC** to 1 and **ST\_RSRC** to zero. Launch **ALU OR** instruction.

- **memcopy in reverse order**: same as above, but set **R[N.DST]** pointer to the end of memory block and **ST\_RDST** to -1;

- **memset**: same as memcopy, but set **R[N.RS]** and **ST\_RRS** to zero.

### PUSH/POP
**PUSH/POP** can be used to perform register data move, like **PUSH R2; POP IP**, which assigns **R2** to **IP**. Instruction decoder can optimize it out and assign values directly and avoid two memory operations.


### JMP/CALL
**JMP** and **CALL** have short and long forms:

- **Short form:** **JMP\_Stride** must be set to known small value (1, 2 or 4) and effective relative address is calculated by multiplying **JMP\_Stride\*offset**, so maximum range could be from 4\*-8 to 4\*7, so -32 to 28 bytes in case of stride = 4; If bigger **JMP\_Stride** is used, better range is possible, but code density will suffer. Gaps could be filled by NOP.
In stright loops, if **condition** and **JMP\_Stride** are the same, **JMP**/**CALL** instruction can be just one byte.

- **Long form:** **JMP**\**_Stride** is set to effective relative address, and offset is set to 1. But **JMP\_Stride** will need to be adjusted for every **JMP**/**CALL**.

### IN/OUT

Format Register structure:
0-2: Format (HEX, DEC, Signed DEC, Binary, Float, Raw).
3: Leading Zeros (0 = minimal, 1 = fixed).
4-5: Length (nibble, byte, half-word, word).
6-8: Precision (0-7 decimal places).

// Format Type (bits 0-2)
typedef enum {
    FMT_RAW         = 0x0,  // 000: Raw bytes
    FMT_HEX         = 0x1,  // 001: Hexadecimal (unsigned)
    FMT_DEC         = 0x2,  // 010: Decimal (unsigned)
    FMT_BINARY      = 0x3,  // 011: Binary (unsigned)
    FMT_FLOAT       = 0x4,  // 100: Float (signed, IEEE 754)
    FMT_SIGNED_DEC  = 0x5,  // 101: Signed Decimal (two's complement)
    FMT_RESERVED1   = 0x6,  // 110: Reserved
    FMT_RESERVED2   = 0x7   // 111: Reserved
} FmtType;

// Leading Zeros (bit 3)
typedef enum {
    FMT_LEADING_MINIMAL = 0x0,  // 0: Strip leading zeros
    FMT_LEADING_FIXED   = 0x8   // 1: Include leading zeros (shifted to bit 3)
} FmtLeadingZeros;

// Length (bits 4-5)
typedef enum {
    FMT_LENGTH_NIBBLE    = 0x00,  // 00: 4 bits
    FMT_LENGTH_BYTE      = 0x10,  // 01: 8 bits (shifted to bits 4-5)
    FMT_LENGTH_HALF_WORD = 0x20,  // 10: 16 bits
    FMT_LENGTH_WORD      = 0x30   // 11: 32 bits
} FmtLength;

// Precision (bits 6-8)
typedef enum {
    FMT_PRECISION_0 = 0x000,  // 000: 0 decimal places
    FMT_PRECISION_1 = 0x040,  // 001: 1 decimal place (shifted to bits 6-8)
    FMT_PRECISION_2 = 0x080,  // 010: 2 decimal places
    FMT_PRECISION_3 = 0x0C0,  // 011: 3 decimal places
    FMT_PRECISION_4 = 0x100,  // 100: 4 decimal places
    FMT_PRECISION_5 = 0x140,  // 101: 5 decimal places
    FMT_PRECISION_6 = 0x180,  // 110: 6 decimal places
    FMT_PRECISION_7 = 0x1C0   // 111: 7 decimal places
} FmtPrecision;




## Assembly Sugar

### LI \<reg>, \<val>
Macro to set any register with arbitrary length immediate value.
Translates into a sequence of RS and LI instructions to load a multi-byte value into a register.
Example:
```
 LI R0, 0x12345678 
```
Translates to:
```
RS R0; LI 8; LI 7; LI 6; LI 5; LI 4; LI 3; LI 2; LI 1 
```

### ADT \<type>
Macro to set ALU Data Type (N.ADT), see table "ALU Data Type Selector".
Translates to:
```
RS FL; NS N.ADT; LI <type>;
```

### CS \<condition>
Macro to set condition to N.BCS, see "Branch Condition Selector" table.
Translates to:
```
RS FL; NS N.BCS; LI <condition>;
```

### SRC \<reg>
Macro to set source register in N.SRC.
Translates to:
```
RS FL; NS N.SRC; LI <reg>;
```

### DST \<reg>
Macro to set destination register in N.DST.
Translates to:
```
RS FL; NS N.DST; LI <reg>;
```

### JS \<JMP\_stride>
Macro to set JMP stride value into R12 register. Same as LI R12, \<JMP\_stride>

### JMP \<offset|\@label> [if \<contition>]

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


### Vector

To simplyfy Vector operations, we define VECTOR macro, with following syntax:
```
 VR type[vl] [R[N.DST]|stride] = ([R[N.RS]|stride] <op> [R[N.SRC]|stride])
```
Where:
- type: data type, uint8 ecc;
- vl: vector length, uint bit length of HWS;
- op: ALU operation, see ALU Operation Mode Selector table;
- stride: distance between operands in memory,  uint bit length of HWS;

Note: when registers are not surrounded by [], it means that is has stride=0, and acts as a constant or accumulator. In case of accumulator, R[N.DST+1] is used to keep upper part of result, in case of R[N.DST] overflow.

Note: for operation both arithmetic symbols (+,-,\*,/, <<, >>) and mnemonics (ADD, SUB, MUL, DIV, SHL, SHR and rest from ALU Operation Mode Selector table) can be used.

Example: to sum 8 bytes referenced by R[N.RS] and R[N.SRC] pointers, with increment by one and save results in memory referenced by R[N.DST], decremented by 4 (reverse order):
```
 VR uint8[8] ([R[N.DST]|-4] = [R[N.RS]] + 32)
```
Note: when stride is not specified, it defaults to data type size, ie sizeof(uint8) = 1;

Which translates to:
```
LI VR_CTRL, 8 << HWS | -4; // vl = 8, Stride\_r2 = -4
LI VR_STRIDE, 1; Stride\_r0 = 1, Stride\_r1 = 0
LI R1, 32; // load constant in R1
ADT uint8;
ALU ADD; // mapped from ‘+’ symbol
```

In case of LOOKUP operation, VR instruction searches for slice at [R[N.RS]] in vector at [R[N.SRC]]. VL is set to vector length, ST\_RRS to the length of slice and ST\_RSRC is not used. LOOKUP returns value of -1 in R[N.DST], if match not found, or positive index of matched slice. Stride_R2 must be 1 for forward search and -1 for reverse search. ADT uint8 set data type to char, but in general any data type can be used.

To find next occurrence, just repeat ALU LOOKUP instruction, it will return next occurrence, if any.

Example: to find index of substring “the” in string “Here they come”
```
VR (R[N.DST] = LOOKUP("Here they come", "the"))
```
Which translates to:
```
LI VR_CTRL, sizeof(s0001) << HWS; // vl = string size, stride_r2 = 0
LI VR_STRIDE, sizeof(s0002); stride_r0 = size of substring, stride_r1 = 0
LI R1, @s0001; // load string address in R1
LI R0, @s0002; // load substring address in R0
ADT uint8;     // set type to char
ALU LOOKUP;    // perform search
s0001: alloc uint8[] = "Here they come"
s0002: uint8[] = "the"
```
Example: realize memset( buf, sizeof(buf), 0x55) with VR macro:

>VR uint8[sizeof(buf)] (@buf = 0x55);

Which translates to:
```
LI VR_CTRL, sizeof(buf) << HWS | 1; // vl = buf size, stride_r2 = 1
LI VR_STRIDE, 0; stride_r0 and stride\_r1 = 0
LI R2, @buf; // load str address in R2
LI R0, 0x55; // R0 = 0x55
LI R1, 0; // R1 = 0
ADT uint8;
ALU OR
```

### Assembly Notes

### Unconditional Rule:

If no condition is specified for JMP or CALL, the assembler automatically inserts CS 0x0 (unconditional).

### VR Mode:

When VL > 1, the architecture operates in VECTOR mode:

R[N.RS], R[N.SRC], and R[N.DST] are treated as memory pointers.
After each operation, the pointers are incremented by their respective strides.
If a stride is 0, the corresponding register is treated as a constant or accumulator.

### Interrupt Handling:

Managed via memory mapped registers, to be defined.

INT triggers a software interrupt.
IRET returns from an interrupt.



## Example Program

Here’s an example program that demonstrates the use of assembly sugar:
```
  // VR memory tests
  JS 1;
  JMP @start;
  
  buf: alloc uint8[32] = {};
  
start:
  // zero buf memory
  VR uint8[sizeof(buf)] (@buf = 0);
  //check if buf has all zeros, by summing all bytes
  VR uint8[sizeof(buf)] (R2 = @buf + 0);
  PUSH R2
  POP R0
  LI R1, 0
  VR uint32 (R0 CMP R1)
  JMP @ok if Equal
  
  fail:
  // test failed
  JMP @fail;
  
  ok:
  // everything is ok
  JMP @ok
  ```
  End
  
