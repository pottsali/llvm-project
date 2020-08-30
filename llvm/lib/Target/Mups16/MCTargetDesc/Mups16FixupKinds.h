//===-- Mups16FixupKinds.h - Mups16 Specific Fixup Entries ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16FIXUPKINDS_H
#define LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16FIXUPKINDS_H

#include "llvm/MC/MCFixup.h"
// Mups16 //
// Mups16 //#undef Mups16
// Mups16 //
namespace llvm {
namespace Mups16 {

// This table must be in the same order of
// MCFixupKindInfo Infos[Mups16::NumTargetFixupKinds]
// in Mups16AsmBackend.cpp.
//
enum Fixups {
  // A 32 bit absolute fixup.
  fixup_32 = FirstTargetFixupKind,
// Mups16 //  // A 10 bit PC relative fixup.
// Mups16 //  fixup_10_pcrel,
// Mups16 //  // A 16 bit absolute fixup.
// Mups16 //  fixup_16,
// Mups16 //  // A 16 bit PC relative fixup.
// Mups16 //  fixup_16_pcrel,
// Mups16 //  // A 16 bit absolute fixup for byte operations.
// Mups16 //  fixup_16_byte,
// Mups16 //  // A 16 bit PC relative fixup for command address.
// Mups16 //  fixup_16_pcrel_byte,
// Mups16 //  // A 10 bit PC relative fixup for complicated polymorphs.
// Mups16 //  fixup_2x_pcrel,
// Mups16 //  // A 16 bit relaxable fixup.
// Mups16 //  fixup_rl_pcrel,
// Mups16 //  // A 8 bit absolute fixup.
// Mups16 //  fixup_8,
// Mups16 //  // A 32 bit symbol difference fixup.
// Mups16 //  fixup_sym_diff,
// Mups16 //
  // Marker
  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};
} // end namespace Mups16
} // end namespace llvm

#endif
