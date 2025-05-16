	JMP @start, Always, st=3
buf:
	alloc u8[7] = {1,2,3}
start:
	RS R.F
	NS M.ADT

	LI ADT.u8
	ALU SUB
	OUT IO.stdout
	JMP @start, st=3
