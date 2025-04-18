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

  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override
  {
      const static MCFixupKindInfo Infos[Mups16::NumTargetFixupKinds] = {
        // This table must be in the same order of enum in Mups16FixupKinds.h.
        //
        // name            offset bits flags
        {"fixup_lo8",           0,   8, 0},
        {"fixup_hi8",           0,   8, 0},
        {"fixup_br8",           0,   8, MCFixupKindInfo::FKF_IsPCRel},
        {"fixup_j11",           0,  11, MCFixupKindInfo::FKF_IsPCRel},
      };
      static_assert((array_lengthof(Infos)) == Mups16::NumTargetFixupKinds,
                    "Not all fixup kinds added to Infos array");

      if (Kind < FirstTargetFixupKind)
        return MCAsmBackend::getFixupKindInfo(Kind);

      return Infos[Kind - FirstTargetFixupKind];
  }

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
    // These have -2 because PC is already incremented, so encoded offsets need to be 2 less than actual byte offset
    case Mups16::fixup_mups16_br8:
        Value = ((Value - 2) >> 1) & 0xff; break;
    case Mups16::fixup_mups16_j11:
        Value = ((Value - 2) >> 1) & 0x7ff; break;
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
    unsigned NumBytes = 2; // always a single word, for now

    if (!Value)
    {
        // Value is already zero in the output stream, so don't need to change
        // anything.
        return;
    }

    // We need to 'or' in the fixed-up value with the existing values. For everything except jump immediates we could just modify the second byte, but jump offsets are 11 bits, so they affect the first byte too.
    for (unsigned i = 0; i != NumBytes; ++i) {
        // Little endian, so we have to or in the bits from the top down
        unsigned idx = NumBytes - i - 1;
        Data[Fixup.getOffset() + i] |= uint8_t((Value >> (idx*8)) & 0xff);
    }
}

bool Mups16AsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const
{
  if ((Count % 2) != 0)
    return false;

  // The canonical nop on Mups16 is addi r0, r0, 0
  uint64_t NopCount = Count / 2;
  while (NopCount--)
    OS.write("\x00\x00", 2);

  return true;
}

} // end anonymous namespace

MCAsmBackend *llvm::createMups16MCAsmBackend(const Target &T,
                                             const MCSubtargetInfo &STI,
                                             const MCRegisterInfo &MRI,
                                             const MCTargetOptions &Options) {
  return new Mups16AsmBackend(STI, ELF::ELFOSABI_STANDALONE);
}
