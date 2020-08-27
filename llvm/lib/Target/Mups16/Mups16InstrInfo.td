include "Mups16InstrFormats.td"

////////////////////////////////////////////////////////////////////////////////
// Operand definitions
////////////////////////////////////////////////////////////////////////////////

// Signed immediate values
def imm5 : Operand<i8>;
def imm8 : Operand<i8>;

// Unsigned immediate values
def uimm5 : Operand<i8>
{
    let PrintMethod = "printUImm<5>";
}

def uimm8 : Operand<i8>
{
    let PrintMethod = "printUImm<8>";
}

def uimm11 : Operand<i16>
{
    let PrintMethod = "printUImm<11>";
}

// Memory operand
def MemAsmOperand : AsmOperandClass {
  let Name = "Mem";
}

def memsrc : Operand<i16> {
  //let PrintMethod = "printSrcMemOperand";
  let MIOperandInfo = (ops IntReg, imm5);
  let ParserMatchClass = MemAsmOperand;
  //let EncoderMethod = "getMemOpValue";
  //let DecoderMethod = "DecodeMemOperand";
}

def memdst : Operand<i16> {
//  let PrintMethod = "printSrcMemOperand";
  let MIOperandInfo = (ops IntReg, imm5);
  let ParserMatchClass = MemAsmOperand;
//  let EncoderMethod = "getMemOpValue";
//  let DecoderMethod = "DecodeMemOperand";
}

////////////////////////////////////////////////////////////////////////////////
// Instruction definitions
////////////////////////////////////////////////////////////////////////////////

// TODO: where we have an immediate that isn't 8/16 bits etc, we use
// an alias for i8mm/i16imm anyway. It's unclear where the number of bits
// available for the immediate should be recorded. Mips' J instruction uses an
// OtherVT for a 26-bit immediate, Sparc's sethi uses i32imm for a 22-bit
// immediate.

// TODO: JAL/JALR clobber RA - how do we encode that?

//| Name     | Type | Opcode | Fn |  | Bits                   | Asm Example         | Summary
//|----------|------|--------|----|--|------------------------|---------------------| -----------
//| ADDI     | I    | 0      |    |  | `00000 RDD RS1 III II` | `addi  sp, sp, -4 ` | Add 5-bit immediate to `RS1`, store in `RDD` |
def ADDI  : InstMupsI<0, (outs IntReg:$rdd), (ins IntReg:$rs1, imm5:$imm5),
                      "addi $rdd, $rs1, $imm5",
                      [/*TODO(set i16:$rdd, (add i16:$rs1, i8:$imm5))*/]>;
//| ADDIU    | I    | 1      |    |  | `00001 RDD RS1 III II` | `addiu sp, sp, 18 ` | Add 5-bit unsigned immediate to `RS1`, store in `RDD` |
def ADDIU : InstMupsI<1, (outs IntReg:$rdd), (ins IntReg:$rs1, uimm5:$imm5),
                      "addiu $rdd, $rs1, $imm5",
                      [/*TODO(set i16:$rdd, (add i16:$rs1, i8:$imm5))*/]>;
//| ANDI     | I    | 2      |    |  | `00010 RDD RS1 III II` | `andi  r1, r2, 0x1` | Bitwise AND of 5-bit immediate with `RS1`, store in `RDD` |
def ANDI  : InstMupsI<2, (outs IntReg:$rdd), (ins IntReg:$rs1, uimm5:$imm5),
                      "andi $rdd, $rs1, $imm5",
                      [/*TODO(set i16:$rdd, (and i16:$rs1, i8:$imm5))*/]>;
//| ORI      | I    | 3      |    |  | `00011 RDD RS1 III II` | `ori   r1, r2, 0x1` | Bitwise OR of 5-bit immediate with `RS1`, store in `RDD` |
def ORI   : InstMupsI<3, (outs IntReg:$rdd), (ins IntReg:$rs1, uimm5:$imm5),
                      "ori $rdd, $rs1, $imm5",
                      [/*TODO(set i16:$rdd, (or i16:$rs1, i8:$imm5))*/]>;
//| XORI     | I    | 4      |    |  | `00100 RDD RS1 III II` | `xori  r1, r2, 0x1` | Bitwise XOR of 5-bit immediate with `RS1`, store in `RDD` |
def XORI  : InstMupsI<4, (outs IntReg:$rdd), (ins IntReg:$rs1, uimm5:$imm5),
                      "xori $rdd, $rs1, $imm5",
                      [/*TODO(set i16:$rdd, (xor i16:$rs1, i8:$imm5))*/]>;
//| NORI     | I    | 5      |    |  | `00101 RDD RS1 III II` | `nori  r1, r2, 0x1` | Bitwise NOR of 5-bit immediate with `RS1`, store in `RDD` |
def NORI  : InstMupsI<5, (outs IntReg:$rdd), (ins IntReg:$rs1, uimm5:$imm5),
                      "nori $rdd, $rs1, $imm5",
                      [/*TODO*/]>;
//| J        | J    | 6      |    |  | `00110 III III III II` | `j     0x07fe     ` | Unconditional jump to 11-bit immediate address (top 5 bits of destination come from existing `PC`) |
def J     : InstMupsJ<6, (ins uimm11:$imm11),
                      "j $imm11",
                      [/*TODO*/]>;
//| JAL      | J    | 7      |    |  | `00111 III III III II` | `jal   0x07fe     ` | Store `PC` into `RA` and jump to 11-bit immediate address (top 5 bits of destination come from existing `PC`) |
def JAL   : InstMupsJ<7, (ins uimm11:$imm11),
                      "jal $imm11",
                      [/*TODO*/]>;
//| JR       | II   | 8      |    |  | `01000 RDD III III II` | `jr    ra, 0      ` | Add 8-bit immediate to `RDD` and jump to result |
def JR  : InstMupsII1<8, (ins memsrc:$src),
                      "jr  $src",
                      [/*TODO*/]>;
//| JALR     | II   | 9      |    |  | `01001 RDD III III II` | `jalr  r1, 0x2    ` | Store `PC` into `RA`, add 8-bit immediate to `RS1` and jump to result |
def JALR: InstMupsII1<9, (ins memsrc:$src),
                      "jalr $src",
                      [/*TODO*/]>;
//| LB       | I    | 10     |    |  | `01010 RDD RS1 III II` | `lb    r1, [6]sp  ` | Add 5-bit immediate to `RS1`, and load single byte from resulting memory address into `RDD`, with sign extension of bit 7 |
def LB  : InstMupsID1<10, (outs IntReg:$rdd), (ins memsrc:$src),
                      "lb $rdd, $src",
                      [/*TODO*/]>;
//| LBU      | I    | 11     |    |  | `01011 RDD RS1 III II` | `lbu   r1, [6]sp  ` | Add 5-bit immediate to `RS1`, and load single byte from resulting memory address into `RDD`, with top 8 bits set to 0 |
def LBU : InstMupsID1<11, (outs IntReg:$rdd), (ins memsrc:$src),
                      "lbu $rdd, $src",
                      [/*TODO*/]>;
//| LW       | I    | 12     |    |  | `01100 RDD RS1 III II` | `lw    r1, [6]sp  ` | Add 5-bit immediate to `RS1`, and load two-byte word from resulting memory address into `RDD` |
def LW  : InstMupsID1<12, (outs IntReg:$rdd), (ins memsrc:$src),
                      "lw $rdd, $src",
                      [/*TODO*/]>;
//| LI       | II   | 13     |    |  | `01101 RDD III III II` | `li    r1, 0x7e   ` | Load 8-bit immediate value into `RDD`, with sign-extension of bit 7 |
def LI  : InstMupsIID<13, (outs IntReg:$rdd), (ins imm8:$imm8),
                      "li $rdd, $imm8",
                      [/*TODO*/]>;
//| LIU      | II   | 14     |    |  | `01110 RDD III III II` | `liu   r1, 0xff   ` | Load 8-bit immediate value into `RDD`, with top 8 bits set to 0 |
def LIU : InstMupsIID<14, (outs IntReg:$rdd), (ins uimm8:$imm8),
                      "li $rdd, $imm8",
                      [/*TODO*/]>;
//| LUI      | II   | 15     |    |  | `01111 RDD III III II` | `lui   r1, 0x01   ` | Load 8-bit immediate value into upper 8 bits of `RDD`. Lower 8 bits are unchanged |
def LUI : InstMupsIID<15, (outs IntReg:$rdd), (ins imm8:$imm8),
                      "lui $rdd, $imm8",
                      [/*TODO*/]>;
//| SB       | I    | 16     |    |  | `10000 RS1 RS2 III II` | `sb    [6]sp, r1  ` | Add 5-bit immediate to `RS1`, and store low byte in `RS2` to resulting memory address |
def SB   : InstMupsI2<16, (ins memdst:$dst, IntReg:$rs2),
                      "sb $dst, $rs2",
                      [/*TODO*/]>;
//| SW       | I    | 17     |    |  | `10001 RS1 RS2 III II` | `sw    [6]sp, r1  ` | Add 5-bit immediate to `RS1`, and store two-byte word `RS2` to resulting memory address |
def SW   : InstMupsI2<17, (ins memdst:$dst, IntReg:$rs2),
                      "sw $dst, $rs2",
                      [/*TODO*/]>;
//| _unused_ |      | 18     |    |  | `10010               ` |                     | |
//| _unused_ |      | 19     |    |  | `10011               ` |                     | |
//| BZ       | II   | 20     |    |  | `10100 RS1 III III II` | `bz    r1, -26    ` | Branch to `PC` plus 8-bit immediate if `RS1` is zero |
def BZ  : InstMupsII1<20, (ins IntReg:$rs1, imm8:$imm8),
                      "bz $rs1, $imm8",
                      [/*TODO*/]>;
//| BNZ      | II   | 21     |    |  | `10101 RS1 III III II` | `bnz   r1, -26    ` | Branch to `PC` plus 8-bit immediate if `RS1` is not zero |
def BNZ : InstMupsII1<20, (ins IntReg:$rs1, imm8:$imm8),
                      "bnz $rs1, $imm8",
                      [/*TODO*/]>;
//| _unused_ |      | 22     |    |  | `10110               ` |                     | |
//| _unused_ |      | 23     |    |  | `10111               ` |                     | |
//| ADD      | R    | 24     | 0  |  | `11000 RDD RS1 RS2 00` | `add   r1, r2, r3 ` | Add `RS1` to `RS2`, store result in `RDD` |
def ADD : InstMupsRD2<24,      0, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "add $rdd, $rs1, $rs2",
                      [(set i16:$rdd, (add i16:$rs1, i16:$rs2))]>;
//| _unused_ |      | 24     | 1  |  | `10111               ` |                     | |
//| SUB      | R    | 24     | 2  |  | `11000 RDD RS1 RS2 10` | `sub   r1, r2, r3 ` | Subtract `RS2` from `RS1`, store result in `RDD` |
def SUB : InstMupsRD2<24,      2, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "sub $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| SUBU     | R    | 24     | 3  |  | `11000 RDD RS1 RS2 11` | `subu  r1, r2, r3 ` | Subtract `RS2` from `RS1`, store result in `RDD`, unsgigned |
def SUBU: InstMupsRD2<24,      3, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "subu $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| AND      | R    | 25     | 0  |  | `11001 RDD RS1 RS2 00` | `and   r1, r2, r3 ` | Bitwise AND of `RS1` with `RS2`, store result in `RDD` |
def AND : InstMupsRD2<25,      0, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "and $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| OR       | R    | 25     | 1  |  | `11001 RDD RS1 RS2 01` | `or    r1, r2, r3 ` | Bitwise OR of `RS1` with `RS2`, store result in `RDD` |
def OR  : InstMupsRD2<25,      1, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "or $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| XOR      | R    | 25     | 2  |  | `11001 RDD RS1 RS2 10` | `xor   r1, r2, r3 ` | Bitwise XOR of `RS1` with `RS2`, store result in `RDD` |
def XOR : InstMupsRD2<25,      2, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "xor $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| NOR      | R    | 25     | 3  |  | `11001 RDD RS1 RS2 11` | `nor   r1, r2, r3 ` | Bitwise NOR of `RS1` with `RS2`, store result in `RDD` |
def NOR : InstMupsRD2<25,      3, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "nor $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| SLL      | R    | 26     | 0  |  | `11010 RDD RS1 RS2 00` | `sll   r1, r2, r3 ` | Shift contents of `RS1` left by amount given in bottom 5 bits of `RS2`, shifting in zero, store result in `RDD` |
def SLL : InstMupsRD2<26,      0, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "sll $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| SRL      | R    | 26     | 1  |  | `11010 RDD RS1 RS2 01` | `srl   r1, r2, r3 ` | Shift contents of `RS1` right by amount given in bottom 5 bits of `RS2`, shifting in zero, store result in `RDD` |
def SRL : InstMupsRD2<26,      1, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "srl $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| SRA      | R    | 26     | 2  |  | `11010 RDD RS1 RS2 10` | `sra   r1, r2, r3 ` | Shift contents of `RS1` left by amount given in bottom 5 bits of `RS2`, shifting in bit 15 of `RS1`, store result in `RDD` |
def SRA : InstMupsRD2<26,      2, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "sra $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| _unused_ | R    | 26     | 3  |  | `11010               ` |                     | |
//| SEQ      | R    | 27     | 0  |  | `11011 RDD RS1 RS2 00` | `seq   r1, r2, r3 ` | Set `RDD` to 1 if `RS1 == RS2`, 0 otherwise |
def SEQ : InstMupsRD2<27,      0, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "seq $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| SNE      | R    | 27     | 1  |  | `11011 RDD RS1 RS2 01` | `sne   r1, r2, r3 ` | Set `RDD` to 1 if `RS1 != RS2`, 0 otherwise |
def SNE : InstMupsRD2<27,      1, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "sne $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| SLT      | R    | 27     | 2  |  | `11011 RDD RS1 RS2 10` | `slt   r1, r2, r3 ` | Set `RDD` to 1 if `RS1 < RS2`, 0 otherwise, signed comparison |
def SLT : InstMupsRD2<27,      2, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "slt $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| SLTU     | R    | 27     | 3  |  | `11011 RDD RS1 RS2 11` | `sltu  r1, r2, r3 ` | Set `RDD` to 1 if `RS1 < RS2`, 0 otherwise, unsigned comparison |
def SLTU: InstMupsRD2<27,      3, (outs IntRegs:$rdd), (ins IntRegs:$rs1, IntRegs:$rs2),
                      "sltu $rdd, $rs1, $rs2",
                      [/*TODO*/]>;
//| _unused_ |      | 28     |    |  | `11100               ` |                     | |
//| _unused_ |      | 29     |    |  | `11101               ` |                     | |
//| RFE      | R    | 30     | 0  |  | `11110 000 000 000 00` | `rfe              ` | Return from exception. Restore flags from values in `S1`, set page table index from value in `S2`, move `SPC` to `PC` |
def RFE   : InstMupsR<30,      0, (outs), (ins),
                      "rfe",
                      [/*TODO*/]>;
//| SYSCALL  | R    | 30     | 1  |  | `11110 RS1 000 000 01` | `syscall r1       ` | Invoke system call given in `RS1`. Syscall args are passed in user registers |
def SYSCALL:InstMupsR1<30,     1, (ins IntRegs:$rs1),
                      "syscall $rs1",
                      []>;
//| MFS      | R    | 30     | 2  |  | `11110 RDD RS1 000 10` | `mfs r1, s1       ` | Move from system reg. Move contents of `RS1` to register `RDD`+8 |
def MFS  : InstMupsR2<30,      2, (ins IntRegs:$rs1, SysReg:$rs2),
                      "mfs $rs1, $rs2",
                      []>;
//| MTS      | R    | 30     | 3  |  | `11110 RDD RS1 000 11` | `mts s1, r1       ` | Move to system reg. Move contents of register `RDD`+8 to `RS1` |
def MTS  : InstMupsR2<30,      3, (ins SysReg:$rs1, IntRegs:$rs2),
                      "mts $rs1, $rs2",
                      []>;
//| PTRI     | R    | 31     | 0  |  | `11111 RDD 000 000 00` | `ptri r1          ` | Read current page table index into `RDD` |
def PTRI : InstMupsRD<31,      0, (outs IntRegs:$rdd), (ins),
                      "ptri $rdd",
                      []>;
//| PTSI     | R    | 31     | 1  |  | `11111 RS1 000 000 01` | `ptsi r1          ` | Set page table index to contents of `RS1` |
def PTSI : InstMupsR1<31,      1, (ins IntRegs:$rs1),
                      "ptsi $rs1",
                      []>;
//| PTRE     | R    | 31     | 2  |  | `11111 RDD RS1 000 10` | `ptre r1, r2      ` | Read current page table entry at address given in `RS1` into `RDD` |
def PTRE: InstMupsRD1<31,      2, (outs IntReg:$rdd), (ins IntRegs:$rs1),
                      "ptre $rdd, $rs1",
                      []>;
//| PTSE     | R    | 31     | 3  |  | `11111 RS1 RS2 000 11` | `ptse r1, r2      ` | Set page table entry at address given in `RS1` to contents of `RS2` |
def PTSE : InstMupsR2<31,      3, (ins IntReg:$rs1, IntRegs:$rs2),
                      "ptse $rs1, $rs2",
                      []>;
