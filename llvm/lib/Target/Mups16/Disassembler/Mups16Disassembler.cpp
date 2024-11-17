#include "Mups16.h"

#include "Mups16RegisterInfo.h"
#include "Mups16Subtarget.h"
#include "TargetInfo/Mups16TargetInfo.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/MathExtras.h"

using namespace llvm;

#define DEBUG_TYPE "mups16-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

class Mups16Disassembler : public MCDisassembler {
public:
    Mups16Disassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
    : MCDisassembler(STI, Ctx)
    {
    }

    virtual ~Mups16Disassembler()
    {
    }

    DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                                ArrayRef<uint8_t> Bytes, uint64_t Address,
                                raw_ostream &CStream) const override;
};

} // end anonymous namespace

// Decoder tables for GPR register
static const unsigned RegisterTable[] = {
  MUPS::R0,
  MUPS::R1,
  MUPS::R2,
  MUPS::R3,
  MUPS::R4,
  MUPS::SP,
  MUPS::R5,
  MUPS::RA,
  MUPS::SPC,
  MUPS::S1,
  MUPS::S2,
  MUPS::S3,
  MUPS::TSP,
  MUPS::PC,
  MUPS::FLG,
};

static MCDisassembler *createMups16Disassembler(const Target &T, const MCSubtargetInfo &STI, MCContext &Ctx)
{
    return new Mups16Disassembler(STI, Ctx);
}

extern "C" void LLVMInitializeMups16Disassembler()
{
    // Register the disassembler.
    TargetRegistry::RegisterMCDisassembler(getTheMups16Target(), createMups16Disassembler);
}

//////////////////////////////////////////////////////////////////////////////////
// Decode functions forward declarations, referred to in
// Mups16InstrInfo.td. We need to forward declare these because the generated
// decodeInstruction function will refer to them.
//////////////////////////////////////////////////////////////////////////////////
static DecodeStatus DecodeMemOperand(MCInst &Inst,
                                     unsigned RegNo,
                                     uint64_t Address,
                                     const void *Decoder);

static DecodeStatus DecodeLoad(MCInst &Inst, unsigned Insn,
    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeStore(MCInst &Inst, unsigned Insn,
    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeIntRegsRegisterClass(MCInst &Inst, unsigned RegNo,
    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeSysRegsRegisterClass(MCInst &Inst, unsigned RegNo,
    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeBranchTarget(MCInst &Inst, unsigned Offset,
    uint64_t Address, const void *Decoder);

#include "Mups16GenDisassemblerTables.inc"

/// Read two bytes from the ArrayRef as a big-endian value
static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address, uint64_t &Size, uint32_t &Insn)
{
    // We want to read exactly 2 Bytes of data.
    if (Bytes.size() < 2)
    {
      Size = 0;
      return MCDisassembler::Fail;
    }

    Insn = (Bytes[0] << 8) | Bytes[1];
    return MCDisassembler::Success;
}


DecodeStatus Mups16Disassembler::getInstruction(MCInst &Instr, uint64_t &Size,
    ArrayRef<uint8_t> Bytes, uint64_t Address, raw_ostream &CStream) const
{
    uint32_t Insn;
    DecodeStatus Result;
    Result = readInstruction16(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail)
      return MCDisassembler::Fail;

    // Calling the auto-generated decoder function.
    Result = decodeInstruction(DecoderTableMups16, Instr, Insn, Address, this, STI);
    if (Result != MCDisassembler::Fail)
    {
        Size = 2;
        return Result;
    }

    return MCDisassembler::Fail;
}

//////////////////////////////////////////////////////////////////////////////////
// Decode functions implementations
//////////////////////////////////////////////////////////////////////////////////

// Helpers
template <unsigned Idx>
static unsigned getRegField(unsigned Instruction)
{
    return Instruction >> (11 - (Idx * 3)) & 0x8;
}

template <unsigned Bits>
static unsigned getImmField(unsigned Instruction)
{
    int16_t Ret = Instruction << (16 - Bits);
    return Ret >> (16 - Bits);
}

template <unsigned Bits>
static unsigned getUImmField(unsigned Instruction)
{
    return Instruction & ((1 << Bits) - 1);
}

// Decode the whole of a load instruction, since I can't work out how to get the memory operand to decode automatically
static DecodeStatus DecodeLoad(MCInst &Inst, unsigned Insn,
    uint64_t Address, const void *Decoder)
{
    Inst.addOperand(MCOperand::createReg(RegisterTable[getRegField<0>(Insn)]));
    Inst.addOperand(MCOperand::createReg(RegisterTable[getRegField<1>(Insn)]));
    Inst.addOperand(MCOperand::createImm(getImmField<5>(Insn)));
    return MCDisassembler::Success;
}
static DecodeStatus DecodeStore(MCInst &Inst, unsigned Insn,
    uint64_t Address, const void *Decoder)
{
    Inst.addOperand(MCOperand::createReg(RegisterTable[getRegField<0>(Insn)]));
    Inst.addOperand(MCOperand::createReg(RegisterTable[getRegField<1>(Insn)]));
    Inst.addOperand(MCOperand::createImm(getImmField<5>(Insn)));
    return MCDisassembler::Success;
}

static DecodeStatus DecodeMemOperand(MCInst &Inst, unsigned Insn,
    uint64_t Address, const void *Decoder)
{
    // Memory operands consist of
    //  - a base register (in bits 8-10)
    //  - an offset (in bits 0-7)
    Inst.addOperand(MCOperand::createReg(RegisterTable[getRegField<0>(Insn)]));
    Inst.addOperand(MCOperand::createImm(getImmField<5>(Insn)));
    return MCDisassembler::Success;
}

// Note: if you don't override the DecoderMethod then the auto-generated code
// seems to call functions based on the operand types
static DecodeStatus DecodeIntRegsRegisterClass(MCInst &Inst, unsigned RegNo,
    uint64_t Address, const void *Decoder)
{
    Inst.addOperand(MCOperand::createReg(RegisterTable[RegNo]));
    return MCDisassembler::Success;
}

static DecodeStatus DecodeSysRegsRegisterClass(MCInst &Inst, unsigned RegNo,
    uint64_t Address, const void *Decoder)
{
    Inst.addOperand(MCOperand::createReg(RegisterTable[RegNo+8]));
    return MCDisassembler::Success;
}

static DecodeStatus DecodeBranchTarget(MCInst &Inst, unsigned Offset,
    uint64_t Address, const void *Decoder)
{
    Inst.addOperand(MCOperand::createImm((SignExtend32<8>(Offset) * 2) + 2));
    return MCDisassembler::Success;
}

