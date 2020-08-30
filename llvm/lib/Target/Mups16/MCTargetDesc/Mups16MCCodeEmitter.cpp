//===-- Mups16MCCodeEmitter.cpp - Convert Mups16 code to machine code -----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the Mups16MCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "Mups16.h"
#include "MCTargetDesc/Mups16MCTargetDesc.h"
#include "MCTargetDesc/Mups16FixupKinds.h"

// MSP430 //#include "llvm/ADT/APFloat.h"
// MSP430 //#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCCodeEmitter.h"
// MSP430 //#include "llvm/MC/MCContext.h"
// MSP430 //#include "llvm/MC/MCExpr.h"
// MSP430 //#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
// MSP430 //#include "llvm/MC/MCInstrInfo.h"
// MSP430 //#include "llvm/MC/MCRegisterInfo.h"
// MSP430 //#include "llvm/MC/MCSubtargetInfo.h"
// MSP430 //#include "llvm/Support/Endian.h"
// MSP430 //#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mccodeemitter"

namespace llvm {

class Mups16MCCodeEmitter : public MCCodeEmitter {
  MCContext &Ctx;
  MCInstrInfo const &MCII;
// MSP430 //
// MSP430 //  // Offset keeps track of current word number being emitted
// MSP430 //  // inside a particular instruction.
// MSP430 //  mutable unsigned Offset;
// MSP430 //
// MSP430 //  /// TableGen'erated function for getting the binary encoding for an
// MSP430 //  /// instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  /// Returns the binary encoding of operands.
  ///
  /// If an operand requires relocation, the relocation is recorded
  /// and zero is returned.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

// MSP430 //  unsigned getMemOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                         SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                         const MCSubtargetInfo &STI) const;
// MSP430 //
// MSP430 //  unsigned getPCRelImmOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                              SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                              const MCSubtargetInfo &STI) const;
// MSP430 //
// MSP430 //  unsigned getCGImmOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                           SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                           const MCSubtargetInfo &STI) const;
// MSP430 //
// MSP430 //  unsigned getCCOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                        SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                        const MCSubtargetInfo &STI) const;
// MSP430 //
public:
  Mups16MCCodeEmitter(MCContext &ctx, MCInstrInfo const &MCII)
      : Ctx(ctx), MCII(MCII) {}

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;
};

void Mups16MCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
// MSP430 //  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
// MSP430 //  // Get byte count of instruction.
// MSP430 //  unsigned Size = Desc.getSize();
// MSP430 //
// MSP430 //  // Initialize fixup offset
// MSP430 //  Offset = 2;
// MSP430 //
// MSP430 //  uint64_t BinaryOpCode = getBinaryCodeForInstr(MI, Fixups, STI);
// MSP430 //  size_t WordCount = Size / 2;
// MSP430 //
// MSP430 //  while (WordCount--) {
// MSP430 //    support::endian::write(OS, (uint16_t)BinaryOpCode, support::little);
// MSP430 //    BinaryOpCode >>= 16;
// MSP430 //  }
}

unsigned Mups16MCCodeEmitter::getMachineOpValue(const MCInst &MI,
                                                const MCOperand &MO,
                                                SmallVectorImpl<MCFixup> &Fixups,
                                                const MCSubtargetInfo &STI) const {
// MSP430 //  if (MO.isReg())
// MSP430 //    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());
// MSP430 //
// MSP430 //  if (MO.isImm()) {
// MSP430 //    Offset += 2;
// MSP430 //    return MO.getImm();
// MSP430 //  }
// MSP430 //
// MSP430 //  assert(MO.isExpr() && "Expected expr operand");
// MSP430 //  Fixups.push_back(MCFixup::create(Offset, MO.getExpr(),
// MSP430 //      static_cast<MCFixupKind>(Mups16::fixup_16_byte), MI.getLoc()));
// MSP430 //  Offset += 2;
  return 0;
}

// MSP430 //unsigned Mups16MCCodeEmitter::getMemOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                                            SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                                            const MCSubtargetInfo &STI) const {
// MSP430 //  const MCOperand &MO1 = MI.getOperand(Op);
// MSP430 //  assert(MO1.isReg() && "Register operand expected");
// MSP430 //  unsigned Reg = Ctx.getRegisterInfo()->getEncodingValue(MO1.getReg());
// MSP430 //
// MSP430 //  const MCOperand &MO2 = MI.getOperand(Op + 1);
// MSP430 //  if (MO2.isImm()) {
// MSP430 //    Offset += 2;
// MSP430 //    return ((unsigned)MO2.getImm() << 4) | Reg;
// MSP430 //  }
// MSP430 //
// MSP430 //  assert(MO2.isExpr() && "Expr operand expected");
// MSP430 //  Mups16::Fixups FixupKind;
// MSP430 //  switch (Reg) {
// MSP430 //  case 0:
// MSP430 //    FixupKind = Mups16::fixup_16_pcrel_byte;
// MSP430 //    break;
// MSP430 //  case 2:
// MSP430 //    FixupKind = Mups16::fixup_16_byte;
// MSP430 //    break;
// MSP430 //  default:
// MSP430 //    FixupKind = Mups16::fixup_16_byte;
// MSP430 //    break;
// MSP430 //  }
// MSP430 //  Fixups.push_back(MCFixup::create(Offset, MO2.getExpr(),
// MSP430 //    static_cast<MCFixupKind>(FixupKind), MI.getLoc()));
// MSP430 //  Offset += 2;
// MSP430 //  return Reg;
// MSP430 //}
// MSP430 //
// MSP430 //unsigned Mups16MCCodeEmitter::getPCRelImmOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                                                 SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                                                 const MCSubtargetInfo &STI) const {
// MSP430 //  const MCOperand &MO = MI.getOperand(Op);
// MSP430 //  if (MO.isImm())
// MSP430 //    return MO.getImm();
// MSP430 //
// MSP430 //  assert(MO.isExpr() && "Expr operand expected");
// MSP430 //  Fixups.push_back(MCFixup::create(0, MO.getExpr(),
// MSP430 //    static_cast<MCFixupKind>(Mups16::fixup_10_pcrel), MI.getLoc()));
// MSP430 //  return 0;
// MSP430 //}
// MSP430 //
// MSP430 //unsigned Mups16MCCodeEmitter::getCGImmOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                                              SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                                              const MCSubtargetInfo &STI) const {
// MSP430 //  const MCOperand &MO = MI.getOperand(Op);
// MSP430 //  assert(MO.isImm() && "Expr operand expected");
// MSP430 //  
// MSP430 //  int64_t Imm = MO.getImm();
// MSP430 //  switch (Imm) {
// MSP430 //  default:
// MSP430 //    llvm_unreachable("Invalid immediate value");
// MSP430 //  case 4:  return 0x22;
// MSP430 //  case 8:  return 0x32;
// MSP430 //  case 0:  return 0x03;
// MSP430 //  case 1:  return 0x13;
// MSP430 //  case 2:  return 0x23;
// MSP430 //  case -1: return 0x33;
// MSP430 //  }
// MSP430 //}
// MSP430 //
// MSP430 //unsigned Mups16MCCodeEmitter::getCCOpValue(const MCInst &MI, unsigned Op,
// MSP430 //                                           SmallVectorImpl<MCFixup> &Fixups,
// MSP430 //                                           const MCSubtargetInfo &STI) const {
// MSP430 //  const MCOperand &MO = MI.getOperand(Op);
// MSP430 //  assert(MO.isImm() && "Immediate operand expected");
// MSP430 //  switch (MO.getImm()) {
// MSP430 //  case Mups16CC::COND_NE: return 0;
// MSP430 //  case Mups16CC::COND_E:  return 1;
// MSP430 //  case Mups16CC::COND_LO: return 2;
// MSP430 //  case Mups16CC::COND_HS: return 3;
// MSP430 //  case Mups16CC::COND_N:  return 4;
// MSP430 //  case Mups16CC::COND_GE: return 5;
// MSP430 //  case Mups16CC::COND_L:  return 6;
// MSP430 //  default:
// MSP430 //    llvm_unreachable("Unknown condition code");
// MSP430 //  }
// MSP430 //}
// MSP430 //
MCCodeEmitter *createMups16MCCodeEmitter(const MCInstrInfo &MCII,
                                         const MCRegisterInfo &MRI,
                                         MCContext &Ctx) {
  return new Mups16MCCodeEmitter(Ctx, MCII);
}

#include "Mups16GenMCCodeEmitter.inc"

} // end of namespace llvm
