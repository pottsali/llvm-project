//===-- Mups16AsmBackend.cpp - Mups16 Assembler Backend -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Mups16FixupKinds.h"
#include "MCTargetDesc/Mups16MCTargetDesc.h"
// MSP430 //#include "llvm/ADT/APInt.h"
#include "llvm/MC/MCAsmBackend.h"
// MSP430 //#include "llvm/MC/MCAssembler.h"
// MSP430 //#include "llvm/MC/MCContext.h"
// MSP430 //#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
// MSP430 //#include "llvm/MC/MCExpr.h"
// MSP430 //#include "llvm/MC/MCFixupKindInfo.h"
// MSP430 //#include "llvm/MC/MCObjectWriter.h"
// MSP430 //#include "llvm/MC/MCSubtargetInfo.h"
// MSP430 //#include "llvm/MC/MCSymbol.h"
// MSP430 //#include "llvm/MC/MCTargetOptions.h"
// MSP430 //#include "llvm/Support/ErrorHandling.h"
// MSP430 //#include "llvm/Support/raw_ostream.h"
// MSP430 //
using namespace llvm;

namespace {
class Mups16AsmBackend : public MCAsmBackend {
  uint8_t OSABI;

// MSP430 //  uint64_t adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
// MSP430 //                            MCContext &Ctx) const;
// MSP430 //
public:
  Mups16AsmBackend(const MCSubtargetInfo &STI, uint8_t OSABI)
      : MCAsmBackend(support::little), OSABI(OSABI) {}
  ~Mups16AsmBackend() override {}

  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsResolved,
                  const MCSubtargetInfo *STI) const override;

  std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override {
    return createMups16ELFObjectWriter(OSABI);
  }

  bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                            const MCRelaxableFragment *DF,
                            const MCAsmLayout &Layout) const override {
    return false;
  }

// MSP430 //  bool fixupNeedsRelaxationAdvanced(const MCFixup &Fixup, bool Resolved,
// MSP430 //                                    uint64_t Value,
// MSP430 //                                    const MCRelaxableFragment *DF,
// MSP430 //                                    const MCAsmLayout &Layout,
// MSP430 //                                    const bool WasForced) const override {
// MSP430 //    return false;
// MSP430 //  }
// MSP430 //
  unsigned getNumFixupKinds() const override {
    return Mups16::NumTargetFixupKinds;
  }
// MSP430 //
// MSP430 //  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override {
// MSP430 //    const static MCFixupKindInfo Infos[Mups16::NumTargetFixupKinds] = {
// MSP430 //      // This table must be in the same order of enum in Mups16FixupKinds.h.
// MSP430 //      //
// MSP430 //      // name            offset bits flags
// MSP430 //      {"fixup_32",            0, 32, 0},
// MSP430 //      {"fixup_10_pcrel",      0, 10, MCFixupKindInfo::FKF_IsPCRel},
// MSP430 //      {"fixup_16",            0, 16, 0},
// MSP430 //      {"fixup_16_pcrel",      0, 16, MCFixupKindInfo::FKF_IsPCRel},
// MSP430 //      {"fixup_16_byte",       0, 16, 0},
// MSP430 //      {"fixup_16_pcrel_byte", 0, 16, MCFixupKindInfo::FKF_IsPCRel},
// MSP430 //      {"fixup_2x_pcrel",      0, 10, MCFixupKindInfo::FKF_IsPCRel},
// MSP430 //      {"fixup_rl_pcrel",      0, 16, MCFixupKindInfo::FKF_IsPCRel},
// MSP430 //      {"fixup_8",             0,  8, 0},
// MSP430 //      {"fixup_sym_diff",      0, 32, 0},
// MSP430 //    };
// MSP430 //    static_assert((array_lengthof(Infos)) == Mups16::NumTargetFixupKinds,
// MSP430 //                  "Not all fixup kinds added to Infos array");
// MSP430 //  
// MSP430 //    if (Kind < FirstTargetFixupKind)
// MSP430 //      return MCAsmBackend::getFixupKindInfo(Kind);
// MSP430 //  
// MSP430 //    return Infos[Kind - FirstTargetFixupKind];
// MSP430 //  }

  bool mayNeedRelaxation(const MCInst &Inst,
                         const MCSubtargetInfo &STI) const override {
    return false;
  }

  bool writeNopData(raw_ostream &OS, uint64_t Count) const override;
};

// MSP430 //uint64_t Mups16AsmBackend::adjustFixupValue(const MCFixup &Fixup,
// MSP430 //                                            uint64_t Value,
// MSP430 //                                            MCContext &Ctx) const {
// MSP430 //  unsigned Kind = Fixup.getKind();
// MSP430 //  switch (Kind) {
// MSP430 //  case Mups16::fixup_10_pcrel: {
// MSP430 //    if (Value & 0x1)
// MSP430 //      Ctx.reportError(Fixup.getLoc(), "fixup value must be 2-byte aligned");
// MSP430 //
// MSP430 //    // Offset is signed
// MSP430 //    int16_t Offset = Value;
// MSP430 //    // Jumps are in words
// MSP430 //    Offset >>= 1;
// MSP430 //    // PC points to the next instruction so decrement by one
// MSP430 //    --Offset;
// MSP430 //
// MSP430 //    if (Offset < -512 || Offset > 511)
// MSP430 //      Ctx.reportError(Fixup.getLoc(), "fixup value out of range");
// MSP430 //
// MSP430 //    // Mask 10 bits
// MSP430 //    Offset &= 0x3ff;
// MSP430 //
// MSP430 //    return Offset;
// MSP430 //  }
// MSP430 //  default:
// MSP430 //    return Value;
// MSP430 //  }
// MSP430 //}
// MSP430 //
void Mups16AsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                  const MCValue &Target,
                                  MutableArrayRef<char> Data,
                                  uint64_t Value, bool IsResolved,
                                  const MCSubtargetInfo *STI) const {
// MSP430 //  Value = adjustFixupValue(Fixup, Value, Asm.getContext());
// MSP430 //  MCFixupKindInfo Info = getFixupKindInfo(Fixup.getKind());
// MSP430 //  if (!Value)
// MSP430 //    return; // Doesn't change encoding.
// MSP430 //
// MSP430 //  // Shift the value into position.
// MSP430 //  Value <<= Info.TargetOffset;
// MSP430 //
// MSP430 //  unsigned Offset = Fixup.getOffset();
// MSP430 //  unsigned NumBytes = alignTo(Info.TargetSize + Info.TargetOffset, 8) / 8;
// MSP430 //
// MSP430 //  assert(Offset + NumBytes <= Data.size() && "Invalid fixup offset!");
// MSP430 //
// MSP430 //  // For each byte of the fragment that the fixup touches, mask in the
// MSP430 //  // bits from the fixup value.
// MSP430 //  for (unsigned i = 0; i != NumBytes; ++i) {
// MSP430 //    Data[Offset + i] |= uint8_t((Value >> (i * 8)) & 0xff);
// MSP430 //  }
}

bool Mups16AsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
// MSP430 //  if ((Count % 2) != 0)
// MSP430 //    return false;
// MSP430 //
// MSP430 //  // The canonical nop on Mups16 is mov #0, r3
// MSP430 //  uint64_t NopCount = Count / 2;
// MSP430 //  while (NopCount--)
// MSP430 //    OS.write("\x03\x43", 2);
// MSP430 //
  return true;
}

} // end anonymous namespace

MCAsmBackend *llvm::createMups16MCAsmBackend(const Target &T,
                                             const MCSubtargetInfo &STI,
                                             const MCRegisterInfo &MRI,
                                             const MCTargetOptions &Options) {
  return new Mups16AsmBackend(STI, ELF::ELFOSABI_STANDALONE);
}
