include "Mups16InstrFormats.td"

////////////////////////////////////////////////////////////////////////////////
// Operand definitions
////////////////////////////////////////////////////////////////////////////////

// Unsigned immediate values
def uimm5 : Operand<i16>
{
    let PrintMethod = "printUImm<5>";
}


////////////////////////////////////////////////////////////////////////////////
// Instruction definitions
////////////////////////////////////////////////////////////////////////////////
def ADD : InstMupsRD2< 24, 0, (outs IntRegs:$rd), (ins IntRegs:$rs1, IntRegs:$rs2),
                       "add $rd, $rs1, $rs2",
                       [(set i16:$rd, (add i16:$rs1, i16:$rs2))]>;

def SYSCALL : InstMupsR1< 30, 0, (ins IntRegs:$rs1), "syscall $rs1", []>;

//def ADDI : TypeI<0, (outs IntRegs:$rd), (ins IntRegs:$rs, imm5Op:$imm),
//                 "addi $rd, $rs, $imm"

