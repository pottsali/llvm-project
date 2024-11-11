//===-- Mups16ELFObjectWriter.cpp - Mups16 ELF Writer ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Mups16FixupKinds.h"
#include "MCTargetDesc/Mups16MCTargetDesc.h"

#include "MCTargetDesc/Mups16MCTargetDesc.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
class Mups16ELFObjectWriter : public MCELFObjectTargetWriter {
public:
  Mups16ELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_MUPS16,
                              /*HasRelocationAddend*/ true) {}

  ~Mups16ELFObjectWriter() override {}

protected:
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override {
    // Translate fixup kind to ELF relocation type.
    switch (Fixup.getTargetKind()) {
    case FK_Data_1:                   return ELF::R_MUPS16_8;
    case FK_Data_2:                   return ELF::R_MUPS16_16;
    case Mups16::fixup_mups16_lo8:    return ELF::R_MUPS16_LO8;
    case Mups16::fixup_mups16_hi8:    return ELF::R_MUPS16_HI8;
    default:
      llvm_unreachable("Invalid fixup kind");
    }
  }
};
} // end of anonymous namespace

std::unique_ptr<MCObjectTargetWriter>
llvm::createMups16ELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<Mups16ELFObjectWriter>(OSABI);
}
