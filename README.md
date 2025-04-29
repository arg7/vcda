# VD arch: Specification

Code Density Challenge, Very Dense Architecture with 8-bit instruction size, adaptable for any word width, 8, 16, 32, 64 bits and beyond.

## Definitions:

WS: defines CPU word size, ie 32;

HWS: defines half of WS;

## Registers

| Register | Description |
| --- | --- |
| R0–R14 | general purpose registers, one nibble address |
| R15 | ALU_CFG: ALU configuration register |
| R16–R252 | general purpose registers |
| R252 | ITP: Interrupt Table Pointer |
| R253 | BP: Base Pointer |
| R254 | SP: Stack Pointer |
| R255 | IP: Instruction Pointer |


## ALU_IO_CFG (R251)
| Nibble | Description |
| --- | --- |
| 0 | RS, Register Selector, low|
| 1 | RS, Register Selector, high|
| 2 | SRC Reg, low|
| 3 | SRC Reg, high |
| 4 | DST Reg, low |
| 5 | DST Reg, high |
| 6 | NS, Nibble Selector, low |
| 7 | NS, Nibble Selector, high |

Default on Reset: RS=0, SRC = 1, DST=2, NS=0

## ALU_MODE_CFG (R250)
| Nibble | Description |
| --- | --- |
| 0-1 | ADT[8], ALU Data Type | 
| 2-3 | VL[8], Vector Length | 
| 4-7 | ST\_RDST[16], Destination STRIDE |
Default on Reset: 0

## ALU_VR_STRIDES (R249)
| 0-3 | ST\_RRS[16], First ARG STRIDE |
| 4-7 | ST\_RSRC[16], Second ARG STRIDE |
Default on Reset: 0

## BRANCH_CTRL (R248)
| 28-39 | BCS[8] | JMP\_Stride[24] |
Default on Reset: 1

Note: 
> ALU VR\_CTRL register defines two fields, VL (Vector Length) unsigned int of HWS bit size and ST\_RDST as signed int of same size, which set the behavior of storing result of ALU operation.
> ALU VR\_STRIDE register defines two fields, ST\_RRS and ST\_RSRC as signed int of HWS bit size, which sets the behavior of input operands in ALU operation.
>VL is in ALU Data Type units, one byte for ADT.u8, 4 for ADT.u32.
>These registers can be used as general purpose, if ALU is not used. Using INC, DEC, NOT and CMP is ok.


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
| 0x7 | f16 | 16-bit floating-point (half-precision) |
| 0x8 | f32 | 32-bit floating-point (single-precision) |
| 0x9 | f64 | 64-bit floating-point (double-precision) |
| 0xA | i64 | 64-bit signed integer |
| 0xB | fp4 | 4-bit floating-point |
| 0xC | fp8 | 8-bit floating-point |
| 0xD | u1 | 1-bit boolean type |
| 0xE | i4 | 4-bit int type |
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
| 0x7 | MUL | Multiplication |
| 0x8 | DIV | Division |
| 0x9 | LOOKUP | Lookup slice in vector|
| 0xA | LOAD | Loads nimble, byte or word |
| 0xB | STORE | Saves nimble, byte or word |
| 0xC-0xF | Reserved |  |

Note:

LOAD fetches data from memory, where R[N.SRC] is a base pointer and R[N.RS] is index and saves it to R[N.DST];
if VR_CTRL.VL > 0, iterate over VR_CTRL.VL adding VR_CTRL.ST_RDST to R[N.SRC] and N.DST++, loading multiple values to registers.

SAVE same as LOAD by in reverse direction.

Size of data is determined by FMT instruction, in case of NIBBLE, NS instruction can specify which nibble in R[N.DST] will be affected.

## Input/Output Map
| Value | Operation | Description |
| --- | --- | --- |
| 0x0 | STDIO | stdin/out channel  |
| 0x1 | STDERR | stderr channel  |

## FMT (Formatted I/O Specifier)

Used by formatted `IN`/`OUT` instructions.

*   `FMT_RAW` (u1): 1 = formatted output, 0 = raw.
*   `ZERO_PAD` (u1): 1 = Pad with Zero.
*   `PRECISION` (u2): Number of decimals after point.

## Instruction Set

## Instruction Set Summary (Page 0)

*   **Default Instruction Size:** 1 byte.
*   **Instruction Prefixes:** Instructions can be extended to 2, 4, or 8 bytes using prefixes:
    *   `0xC` (OP2): Defines a two-byte instruction form.
    *   `0xD` (OP4): Defines a four-byte instruction form.
    *   `0xE` (OP8): Defines an eight-byte instruction form.
*   **Instruction Pages:** The ISA supports multiple instruction pages. The default is Page 0. The `EXT` instruction is used to switch pages.
*   **Configuration Registers:** Several implicit configuration registers exist, modified by specific instructions (e.g., NOP prefixes, RS, NS, ALU):
    *   `N`: ALU_IO_CFG (Contains `RS` - Register Select, `NS` - Nibble Select, `DST` - Destination Register)
    *   `B`: BRANCH_CTRL (Contains `BCS` - Branch Condition Selector, `JMP_Stride`)
    *   `M`: ALU_MODE_CFG (Contains `ADT` - ALU Data Type)


| Opcode     | Mnemonic | Brief Description                                                    |
| :--------- | :------- | :------------------------------------------------------------------- |
| `0x0 0x0`  | `NOP`    | No Operation. Prefixed forms configure system registers (`N`,`B`,`M`). |
| `0x0 0x1`  | `RET`    | Return from function call (pops IP). Extended forms adjust SP.       |
| `0x0 0x2`  | `IRET`   | Return from interrupt (pops IP). Extended forms adjust SP.         |
| `0x0 0x3`  | `INC`    | Increment register. Extended forms specify register and immediate.   |
| `0x0 0x4`  | `DEC`    | Decrement register. Extended forms specify register and immediate.   |
| `0x0 0x5`  | `NOT`    | Bitwise NOT (invert) register. Extended forms operate on multiple. |
| `0x1`      | `RS`     | Set Register Select (`N.RS`). Extended form allows larger index.     |
| `0x2`      | `NS`     | Set Nibble Select (`N.NS`) within the selected register (`N.RS`).    |
| `0x3`      | `LI`     | Load Immediate value into register/nibble. Extended forms load more. |
| `0x4`      | `JMP`    | Conditional relative jump. Extended forms specify condition/offset.  |
| `0x5`      | `CALL`   | Conditional relative function call. Extended forms specify condition/offset. |
| `0x6`      | `PUSH`   | Push register(s) onto stack. Extended forms push multiple.         |
| `0x7`      | `POP`    | Pop register(s) from stack. Extended forms pop multiple/offset.      |
| `0x8`      | `ALU`    | Perform ALU operation (ADD, SUB, AND etc.). Extended forms specify operands/type. |
| `0x9`      | `INT`    | Call interrupt handler. Extended form allows larger interrupt number. |
| `0xA`      | `IN`     | Input from I/O channel. Extended forms allow formatted input.      |
| `0xB`      | `OUT`    | Output to I/O channel. Extended forms allow formatted output.      |
| `0xF`      | `EXT`    | Select instruction page for the *next* instruction.                |

### NOP (Opcode: `0x0 0x0`)

No Operation. Prefixed versions configure system registers.

*   **1 byte:** `0x0 0x0`
    *   No operation.
*   **2 bytes:** `0xC 0x0 0x0 xx`
    *   Sets `N = ALU_IO_CFG` based on the following byte (`xx`).
*   **4 bytes:** `0xD 0x0 0x0 xx xx xx xx xx`
    *   Sets `B = BRANCH_CTRL` based on the following bytes.
*   **8 bytes:** `0xE 0x0 0x0 xx ... xx`
    *   Sets `M = ALU_MODE_CFG` based on the following bytes.

---

### RET (Opcode: `0x0 0x1`)

Return from function call.

*   **1 byte:** `0x0 0x1`
    *   Pops Instruction Pointer (IP) from stack. Assumes `cnt=0`. `SP -= cnt`, `POP IP`.
*   **2 bytes:** `0xC 0x0 0x1 cnt`
    *   Adds `cnt` (u8) to Stack Pointer (SP) before popping IP. `SP -= cnt`, `POP IP`.
*   **4 bytes:** `0xD 0x0 0x1 cnt`
    *   Same as 2-byte form, `cnt` is u24.
*   **8 bytes:** `0xE 0x0 0x1 cnt xx ... xx`
    *   Same as 2-byte form, `cnt` is u56.

---

### IRET (Opcode: `0x0 0x2`)

Return from Interrupt. Functionally identical to `RET`.

*   **1 byte:** `0x0 0x2`
*   **2 bytes:** `0xC 0x0 0x2 cnt`
*   **4 bytes:** `0xD 0x0 0x2 cnt`
*   **8 bytes:** `0xE 0x0 0x2 cnt xx ... xx`

---

### INC (Opcode: `0x0 0x3`)

Increment register value.

*   **1 byte:** `0x0 0x3`
    *   `R[N.RS]++`. Increments the register selected by `N.RS`.
*   **2 bytes:** `0xC 0x0 0x3 reg`
    *   `R[reg]++`. Increments register `reg` (u4).
*   **4 bytes:** `0xD 0x0 0x3 reg val`
    *   `R[reg] += val`. Adds `val` (u12/i12, depends on `M.ADT`) to register `reg` (u8).
*   **8 bytes:** `0xE 0x0 0x3 reg val`
    *   `R[reg] += val`. Adds `val` (u44/i44, depends on `M.ADT`) to register `reg` (u8).

---

### DEC (Opcode: `0x0 0x4`)

Decrement register value.

*   **1 byte:** `0x0 0x4`
    *   `R[N.RS]--`. Decrements the register selected by `N.RS`.
*   **2 bytes:** `0xC 0x0 0x4 reg`
    *   `R[reg]--`. Decrements register `reg` (u4).
*   **4 bytes:** `0xD 0x0 0x4 reg val`
    *   `R[reg] -= val`. Subtracts `val` (u12/i12, depends on `M.ADT`) from register `reg` (u8).
*   **8 bytes:** `0xE 0x0 0x4 reg val`
    *   `R[reg] -= val`. Subtracts `val` (u44/i44, depends on `M.ADT`) from register `reg` (u8).

---

### NOT (Opcode: `0x0 0x5` - *Note: Source lists `0x0 0x4`, likely a typo*)

***Note:** The provided context lists the opcode for NOT as `0x0 0x4`, which conflicts with DEC. Assuming a sequential assignment, `0x0 0x5` is used here.*

Bitwise NOT (Invert) register value.

*   **1 byte:** `0x0 0x5`
    *   Inverts bits of `R[N.RS]`.
*   **2 bytes:** `0xC 0x0 0x5 reg`
    *   Inverts bits of `R[reg]`, where `reg` is u4.
*   **4 bytes:** `0xD 0x0 0x5 reg cnt`
    *   Inverts bits of registers from `R[reg]` to `R[reg+cnt]`, where `reg` is u8 and `cnt` is u12.
*   **8 bytes:** `0xE 0x0 0x5 reg cnt xx ... xx`
    *   Same as 4-byte form.

---

### RS (Opcode: `0x1`)

Register Select. Sets the primary source/destination register index.

*   **1 byte:** `0x1 reg`
    *   Sets `N.RS = reg` (u4). Resets `N.NS` to 0 if `reg` is different from the previous `N.RS`.
*   **2 bytes:** `0xC 0x1 reg`
    *   Sets `N.RS = reg` (u8). Resets `N.NS` to 0 if `reg` is different from the previous `N.RS`.

---

### NS (Opcode: `0x2`)

Nibble Select. Sets the nibble offset within the selected register (`N.RS`).

*   **1 byte:** `0x2 val`
    *   Sets `N.NS = val` (u4).

---

### LI (Opcode: `0x3`)

Load Immediate. Loads an immediate value into a register nibble/word.

*   **1 byte:** `0x3 val`
    *   `R[N.RS][N.NS++] = val`. Loads `val` (u4/i4, depending on `M.ADT`) into the nibble at `N.NS` within register `N.RS`. Increments `N.NS`.
*   **2 bytes:** `0xC 0x3 reg val`
    *   Sets `N.RS=reg` (u4). `R[reg][N.NS++] = val`. Loads `val` (u4/i4) into nibble `N.NS` of `R[reg]`. Increments `N.NS`.
*   **4 bytes:** `0xD 0x3 reg val`
    *   Sets `N.RS=reg` (u8). `R[reg][N.NS+=4] = val`. Loads `val` (u16/i16) into the word starting at nibble `N.NS` of `R[reg]`. Increments `N.NS` by 4.
*   **8 bytes:** `0xE 0x3 reg val`
    *   Sets `N.RS=reg` (u8). `R[reg][N.NS+=12] = val`. Loads `val` (u48/i48) into the value starting at nibble `N.NS` of `R[reg]`. Increments `N.NS` by 12.

---

### JMP (Opcode: `0x4`)

Conditional Jump (Relative).

*   **1 byte:** `0x4 ofs`
    *   Jumps if condition `B.BCS` is met. `IP = IP + B.JMP_Stride * ofs`. `ofs` is i4.
*   **2 bytes:** `0xC 0x4 bcs ofs`
    *   Jumps if condition `bcs` (u4) is met. `IP = IP + B.JMP_Stride * ofs`. `ofs` is i4.
*   **4 bytes:** `0xD 0x4 bcs ofs`
    *   Jumps if condition `bcs` (u8) is met. `IP = IP + B.JMP_Stride * ofs`. `ofs` is i16.
*   **8 bytes:** `0xE 0x4 bcs ofs`
    *   Jumps if condition `bcs` (u8) is met. `IP = IP + B.JMP_Stride * ofs`. `ofs` is i48.

---

### CALL (Opcode: `0x5`)

Conditional Call (Relative).

*   Same operands and conditions as `JMP`. Pushes the address of the *next* instruction onto the stack before jumping.

*   **1 byte:** `0x5 ofs`
*   **2 bytes:** `0xC 0x5 bcs ofs`
*   **4 bytes:** `0xD 0x5 bcs ofs`
*   **8 bytes:** `0xE 0x5 bcs ofs`

---

### PUSH (Opcode: `0x6`)

Push register(s) onto the stack.

*   **1 byte:** `0x6 reg`
    *   Pushes `R[reg]` (u4) onto the stack.
*   **2 bytes:** `0xC 0x6 reg`
    *   Pushes `R[reg]` (u8) onto the stack.
*   **4 bytes:** `0xD 0x6 reg cnt`
    *   Pushes registers from `R[reg]` to `R[reg+cnt]` onto the stack. `reg` is u8, `cnt` is u8.

*(Note: 8-byte form description not provided in context)*

---

### POP (Opcode: `0x7`)

Pop register(s) from the stack.

*   **1 byte:** `0x7 reg`
    *   Pops from stack into `R[reg]` (u4).
*   **2 bytes:** `0xC 0x7 reg`
    *   Pops from stack into `R[reg]` (u8).
*   **4 bytes:** `0xD 0x7 reg cnt ofs`
    *   Pops `cnt` (u8) registers starting from `R[reg]` (u8). Uses stack address `SP - ofs * size` where `size` depends on `M.ADT`. If `ofs > 0`, SP is *not* changed (allows fetching recently pushed values without altering SP).

*(Note: 8-byte form description not provided in context)*

---

### ALU (Opcode: `0x8`)

Arithmetic Logic Unit operations.

*   **1 byte:** `0x8 op`
    *   Performs `R[N.DST] = R[N.RS] <op> R[N.SRC]`. `op` (u4) is the AOP selector. Data type depends on `M.ADT`.
*   **2 bytes:** `0xC 0x8 op adt`
    *   Same as 1-byte, but first sets `M.ADT = adt` (u4).
*   **4 bytes:** `0xD 0x8 op adt a1 a2 r`
    *   Sets `M.ADT = adt` (u4), `N.RS = a1` (u4), `N.SRC = a2` (u4), `N.DST = r` (u8). Then performs `R[r] = R[a1] <op> R[a2]`.
*   **8 bytes:** `0xE 0x8 op adt a1 a2 r ofs`
    *   Sets `M.ADT = adt` (u8), `N.RS = a1` (u8), `N.SRC = a2` (u8), `N.DST = r` (u8). Then performs the operation. `ofs` (u16) is used specifically by `LOAD`/`STORE` operations (AOP 0xA, 0xB).

---

### INT (Opcode: `0x9`)

Call Interrupt Handler.

*   **1 byte:** `0x9 val`
    *   Calls interrupt handler `val` (u4).
*   **2 bytes:** `0xC 0x9 val`
    *   Calls interrupt handler `val` (u8).

---

### IN (Opcode: `0xA`)

Input from I/O channel.

*   **1 byte:** `0xA ch`
    *   Reads value from I/O channel `ch` (u4, see IO_MAP) into `R[N.RS]`. Sets `R[N.RS+1]` to zero on success.
*   **2 bytes:** `0xC 0xA ch`
    *   Reads from channel `ch` (u8) into `R[N.RS]`. Sets `R[N.RS+1]` to zero on success.
*   **4 bytes:** `0xD 0xA ch reg adt fmt`
    *   Formatted input from channel `ch` (u8) into `R[reg]` (u8). Input type defined by `adt` (u4) according to format `fmt` (u8, see FMT).

---

### OUT (Opcode: `0xB`)

Output to I/O channel.

*   Operands and forms are the same as `IN`, but performs output instead of input.

*   **1 byte:** `0xB ch`
*   **2 bytes:** `0xC 0xB ch`
*   **4 bytes:** `0xD 0xB ch reg adt fmt`

---

### EXT (Opcode: `0xF`)

Extended Instruction Page prefix. Activates a different instruction page for the *next* instruction.

*   **1 byte:** `0xF ipg`
    *   Activates instruction page `ipg` (u4) for the following instruction.
*   **2 bytes:** `0xC 0xF ipg`
    *   Activates instruction page `ipg` (u8) for the following instruction.



### RS
**RS** instruction resets **N.NS** to 0.

### LI
**LI** loads **immediate** value to nibbles starting from **N.NS** of register **N.RS**; **LI** can be chained to load arbitrary constants. If previous instruction was **LI**, it assigns **immediate** to the next nibbles in the register. 
In case of **ADT** pointing to signed type, it also extend **immediate** sign to the all the upper bits of register.

*Optimization:* instruction decoder can detect **RS**, **NS** and **LI** sequences and optimize by assigning to **N.RS** register value combined from **LI** sequence in one cycle.

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

Format structure:

// Format Type (bits 0)
typedef enum {
    FMT_RAW         = 0x0,  // 000: Raw bytes
    FMT_FORMATED    = 0x1
} FmtType;

// Leading Zeros (bit 1)
typedef enum {
    FMT_LEADING_MINIMAL = 0x0,  // 0: Strip leading zeros
    FMT_LEADING_FIXED   = 0x2   // 1: Include leading zeros
} FmtLeadingZeros;

// Precision (bits 2-3)
typedef enum {
    FMT_PRECISION_0 = 0x0,  // 000: 0 decimal places
    FMT_PRECISION_1 = 0x4,  // 001: 1 decimal place 
    FMT_PRECISION_2 = 0x8,  // 010: 2 decimal places
    FMT_PRECISION_3 = 0xC,  // 011: 4 decimal places
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
  
