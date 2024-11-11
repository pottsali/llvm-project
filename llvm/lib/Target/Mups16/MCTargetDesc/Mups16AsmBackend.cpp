//===-- Mups16AsmBackend.cpp - Mups16 Assembler Backend -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Mups16FixupKinds.h"
#include "MCTargetDesc/Mups16MCTargetDesc.h"
#include "llvm/ADT/APInt.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

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
  unsigned getNumFixupKinds() const override
  {
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
                         const MCSubtargetInfo &STI) const override
  {
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
static unsigned adjustFixupValue(const MCFixup &Fixup, uint64_t Value, MCContext &Ctx)
{
    unsigned Kind = Fixup.getKind();
    switch (Kind)
    {
    case FK_Data_1:
    case FK_Data_2:
    case FK_Data_4:
        return Value;
    case Mups16::fixup_mups16_hi8:
        Value = (Value >> 8) & 0xff; break;
    case Mups16::fixup_mups16_lo8:
        Value = Value & 0xff; break;
    default:
      llvm_unreachable("Unhandled fixup kind in Mups16AsmBackend::applyFixup");
    }
    return Value;
}

void Mups16AsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                  const MCValue &Target,
                                  MutableArrayRef<char> Data,
                                  uint64_t Value, bool IsResolved,
                                  const MCSubtargetInfo *STI) const
{
    MCFixupKind Kind = Fixup.getKind();
    MCContext &Ctx = Asm.getContext();
    Value = adjustFixupValue(Fixup, Value, Ctx);

    if (!Value)
    {
        return; // Doesn't change encoding (we already encoded zero)
    }

    // We could get info on which bits change from the fixup, but so far we only
    // have two cases, both of which just change the bottom byte of the
    // instruction word, so we can hard-code this for now. Where do we start in
    Data[1] = Value;
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
