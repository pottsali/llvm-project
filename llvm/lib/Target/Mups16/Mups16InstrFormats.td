
class InstMups<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Instruction
{
    field bits<16> Inst;
    let Namespace = "MUPS";
    let Size = 2;

    bits<5> op;
    let op = opcode;
    let Inst{15-11} = op;         // Op field is high 5 bits

    dag OutOperandList = outs;
    dag InOperandList = ins;
    let AsmString = asmstr;
    let Pattern = pattern;

    let DecoderNamespace = "Mups";

    field bits<16> SoftFail = 0;
}

////////////////////////////////////////////////////////////////////////////////
// J-type instructions, with an 11-bit immediate
////////////////////////////////////////////////////////////////////////////////
class InstMupsJ<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : InstMups<opcode, outs, ins, asmstr, pattern>
{
    bits<11> imm10;
    let Inst{10-0} = imm10;
}

////////////////////////////////////////////////////////////////////////////////
// I-type instructions, with one dest register, one source, and a 5-bit 
// immediate
////////////////////////////////////////////////////////////////////////////////
class InstMupsI<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : InstMups<opcode, outs, ins, asmstr, pattern>
{
    bits<3> rd;
    bits<3> rs1;
    bits<5> imm5;

    let Inst{10-8} = rd;
    let Inst{7-5} = rs1;
    let Inst{4-0} = imm5;
}

////////////////////////////////////////////////////////////////////////////////
// II-type instructions, with one dest register and an 8-bit immediate
////////////////////////////////////////////////////////////////////////////////
class InstMupsII<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : InstMups<opcode, outs, ins, asmstr, pattern>
{
    bits<3> rd;
    bits<8> imm8;

    let Inst{10-8} = rd;
    let Inst{7-0} = imm8;
}

////////////////////////////////////////////////////////////////////////////////
// R-type instructions, with one dest register and two source registers. These
// also have a 2-bit sub-op suffix
////////////////////////////////////////////////////////////////////////////////
class InstMupsR<bits<5> opcode, bits<2> subop, dag outs, dag ins, string asmstr, 
    list<dag> pattern>
  : InstMups<opcode, outs, ins, asmstr, pattern>
{
    bits<2> subop;
    let Inst{1-0} = subop;
}

// R instructions with no output reg, 1 input reg
class InstMupsR1<bits<5> opcode, bits<2> subop, dag ins, string asmstr,
    list<dag> pattern>
  : InstMupsR<opcode, subop, (outs), ins, asmstr, pattern>
{
    bits<3> rs1;
    let Inst{10-8} = rs1;
}

// R instructions with no output reg, 2 input regs
class InstMupsR2<bits<5> opcode, bits<2> subop, dag ins, string asmstr,
    list<dag> pattern>
  : InstMupsR1<opcode, subop, ins, asmstr, pattern>
{
    bits<3> rs2;
    let Inst{7-5} = rs2;
}

// R instructions with output reg
class InstMupsRD<bits<5> opcode, bits<2> subop, dag outs, dag ins, string asmstr,
    list<dag> pattern>
  : InstMupsR<opcode, subop, outs, ins, asmstr, pattern>
{
    bits<3> rd;
    let Inst{10-8} = rd;
}

// R instructions with output reg + 1 input reg
class InstMupsRD1<bits<5> opcode, bits<2> subop, dag outs, dag ins, string asmstr,
    list<dag> pattern>
  : InstMupsRD<opcode, subop, outs, ins, asmstr, pattern>
{
    bits<3> rs1;
    let Inst{7-5} = rs1;
}

// R instructions with output reg + 2 input regs
class InstMupsRD2<bits<5> opcode, bits<2> subop, dag outs, dag ins, string asmstr,
    list<dag> pattern>
  : InstMupsRD1<opcode, subop, outs, ins, asmstr, pattern>
{
    bits<3> rs2;
    let Inst{4-2} = rs2;
}

