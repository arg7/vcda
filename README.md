# VD arch: Specification

Code Density Challenge, Very Dense Architecture with 8-bit instruction size, adaptable for any word width, 8, 16, 32, 64 bits and beyond.

## Definitions:

WS: defines CPU word size, ie 32;

HWS: defines half of WS;

## Registers

| Register | Description |
| --- | --- |
| R0–R6 | general purpose registers |
| R7 | ALU VR\_CTRL: VL[HWS] | ST\_RDST[HWS] |
| R8 | ALU VR\_STRIDE: ST\_RRS[HWS] ST\_RSRC[HWS] |
| R9 | ITBP: Interrupt Table Base Pointer |
| R10 | ICTRL: Pending[HWS] | Masked[HWS] |
| R11 | FL: Status and control flags |
| R12 | JMP\_Stride: Jump stride, defaults to 1 |
| R13 | BP: Base Pointer |
| R14 | SP: Stack Pointer |
| R15 | IP: Instruction Pointer |



Note: 
> ALU VR\_CTRL register defines two fields, VL (Vector Length) unsigned int of HWS bit size and ST\_RDST as signed int of same size, which set the behavior of storing result of ALU operation.
> ALU VR\_STRIDE register defines two fields, ST\_RRS and ST\_RSRC as signed int of HWS bit size, which sets the behavior of input operands in ALU operation.
>VL is in ALU Data Type units, one byte for ADT.u8, 4 for ADT.u32.
>These registers can be used as general purpose, if ALU is not used. Using INC, DEC, NOT and CMP is ok.

## FLAGS (R11)

| Bit(s) | Nibble | Description |
| --- | --- | --- |
| 0-3 | 0 | NS, Nibble Selector, default 0 |
| 4-7 | 1 | RS, Register Selector, default R0 |
| 8-11 | 2 | SRC Reg, default R1 |
| 12-15 | 3 | DST Reg, default R2 |
| 16-19 | 4 | BCS, Branch Condition Selector |
| 20-23 | 5 | ADT, ALU Data Type Selector |
| 24 | 6 | Carry Flag (C) |
| 25 | 6 | Zero Flag (Z) |
| 26 | 6 | Sign Flag (S) |
| 27 | 6 | Overflow Flag (V) |
| 28 | 7 | Parity Flag (P) |
| 29 | 7 | Interrupt (I) |


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
| 0x1 | SUB | Subtraction |
| 0x2 | AND | Bitwise AND |
| 0x3 | OR | Bitwise OR |
| 0x4 | XOR | Bitwise Exclusive OR |
| 0x5 | SHL | Shift Left |
| 0x6 | SHR | Shift Right Logical |
| 0x7 | SAR | Shift Arithmetic Right (preserves sign) |
| 0x8 | MUL | Multiplication |
| 0x9 | DIV | Division |
| 0xA | LOOKUP | Lookup slice in vector|
| 0xB | LOAD | Loads nimble, byte or word |
| 0xC | STORE | Saves nimble, byte or word |
| 0xD-0xF | Reserved |  |

Note:

LOAD fetches data from memory, where R[N.SRC] is a base pointer and R[N.RS] is index and saves it to R[N.DST], if ST_RDST is 0, otherwise to memory pointed by R[N.DST]

SAVE same as LOAD by in reverse direction.

Size of data is determinied by FMT instruction, in case of NIBBLE, NS instruction can specify which nibble in R[N.DST] will be affected.

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
| 0x0 | 0x8 | CMP | Compare R[N.RS] and R[N.SRC] |
| 0x0 | 0x7 | FMT WORD | one word at time |
| 0x0 | 0x8 | FMT BYTE | one byte at time |
| 0x0 | 0x9 | FMT NIBBLE | one nibble at time |
| 0x0 | 0xA | FMT BIN | as bin |
| 0x0 | 0xB | FMT HEX | as hex |
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
In case of **signed LIS**, it also extend i4 sign to the all the upper nibles of register.

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
 VECTOR type[vl] [R[N.DST]|stride] = ([R[N.RS]|stride] <op> [R[N.SRC]|stride])
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
LI VR\_CTRL, 8 << HWS | -4; // vl = 8, Stride\_r2 = -4
LI VR\_STRIDE, 1; Stride\_r0 = 1, Stride\_r1 = 0
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

ITBP (R9) points to the base of the interrupt table.
ICTRL (R10) tracks pending and masked interrupts.

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
  
