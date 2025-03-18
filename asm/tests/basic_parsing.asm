	JMP @start
buf:
	alloc u8[] = {1,2,3}
start:
	RS R.F
	NS N.ADT
	FMT NIBBLE

	LI ADT.u8
	ALU SUB
	OUT IO.tty0
	JMP @start
