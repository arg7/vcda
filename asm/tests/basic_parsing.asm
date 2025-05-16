	JMP @start  //, Always
buf:
	alloc u8[8] = {1,2,3}
start:
	RS R.F
	NS M.ADT

	LI ADT.u8
	ALU SUB
	OUT IO.stdout
	JMP @start
