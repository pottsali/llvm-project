
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
class InstMupsJ<bits<5> opcode, dag ins, string asmstr, list<dag> pattern>
  : InstMups<opcode, (outs), ins, asmstr, pattern>
{
    bits<11> imm11;
    let Inst{10-0} = imm11;
}

////////////////////////////////////////////////////////////////////////////////
// I-type instructions, with one dest register, one source, and a 5-bit 
// immediate
////////////////////////////////////////////////////////////////////////////////
class InstMupsI<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : InstMups<opcode, outs, ins, asmstr, pattern>
{
    bits<5> imm5;
    let Inst{4-0} = imm5;
}

class InstMupsID1<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : InstMupsI<opcode, outs, ins, asmstr, pattern>
{
    bits<3> rdd;
    bits<3> rs1;

    let Inst{10-8} = rdd;
    let Inst{7-5} = rs1;
}

class InstMupsI2<bits<5> opcode, dag ins, string asmstr, list<dag> pattern>
  : InstMupsI<opcode, (outs), ins, asmstr, pattern>
{
    bits<3> rs1;
    bits<3> rs2;
    let Inst{10-8} = rs1;
    let Inst{7-5} = rs2;
}

////////////////////////////////////////////////////////////////////////////////
// II-type instructions, with one dest register and an 8-bit immediate
////////////////////////////////////////////////////////////////////////////////
class InstMupsII<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : InstMups<opcode, outs, ins, asmstr, pattern>
{
    bits<8> imm8;
    let Inst{7-0} = imm8;
}

class InstMupsIID<bits<5> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : InstMupsII<opcode, outs, ins, asmstr, pattern>
{
    bits<3> rdd;
    let Inst{10-8} = rdd;
}

class InstMupsII1<bits<5> opcode, dag ins, string asmstr, list<dag> pattern>
  : InstMupsII<opcode, (outs), ins, asmstr, pattern>
{
    bits<3> rs1;
    let Inst{10-8} = rs1;
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

  let Inst{10-2} = 0; // Some R-instructions have unused bits, set them to 0
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
class InstMupsRD<bits<5> opcode, bits<2> subop, dag outs, dag ins,
                 string asmstr, list<dag> pattern>
    : InstMupsR<opcode, subop, outs, ins, asmstr, pattern>
{
    bits<3> rdd;
    let Inst{10-8} = rdd;
}

// R instructions with output reg + 1 input reg
class InstMupsRD1<bits<5> opcode, bits<2> subop, dag outs, dag ins,
                  string asmstr, list<dag> pattern>
    : InstMupsRD<opcode, subop, outs, ins, asmstr, pattern>
{
    bits<3> rs1;
    let Inst{7-5} = rs1;
}

// R instructions with output reg + 2 input regs
class InstMupsRD2<bits<5> opcode, bits<2> subop, dag outs, dag ins,
                  string asmstr, list<dag> pattern>
    : InstMupsRD1<opcode, subop, outs, ins, asmstr, pattern>
{
    bits<3> rs2;
    let Inst{4-2} = rs2;
}
