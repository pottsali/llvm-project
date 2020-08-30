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
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_NONE,
                              /*HasRelocationAddend*/ true) {}

  ~Mups16ELFObjectWriter() override {}

protected:
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override {
    // Translate fixup kind to ELF relocation type.
    switch (Fixup.getTargetKind()) {
//    case FK_Data_1:                   return ELF::R_Mups16_8;
//    case FK_Data_2:                   return ELF::R_Mups16_16_BYTE;
//    case FK_Data_4:                   return ELF::R_Mups16_32;
//    case Mups16::fixup_32:            return ELF::R_Mups16_32;
//    case Mups16::fixup_10_pcrel:      return ELF::R_Mups16_10_PCREL;
//    case Mups16::fixup_16:            return ELF::R_Mups16_16;
//    case Mups16::fixup_16_pcrel:      return ELF::R_Mups16_16_PCREL;
//    case Mups16::fixup_16_byte:       return ELF::R_Mups16_16_BYTE;
//    case Mups16::fixup_16_pcrel_byte: return ELF::R_Mups16_16_PCREL_BYTE;
//    case Mups16::fixup_2x_pcrel:      return ELF::R_Mups16_2X_PCREL;
//    case Mups16::fixup_rl_pcrel:      return ELF::R_Mups16_RL_PCREL;
//    case Mups16::fixup_8:             return ELF::R_Mups16_8;
//    case Mups16::fixup_sym_diff:      return ELF::R_Mups16_SYM_DIFF;
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
