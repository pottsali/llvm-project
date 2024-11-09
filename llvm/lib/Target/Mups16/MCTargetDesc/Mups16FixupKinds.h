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
  // 8-bit fixup corresponding to lo(foo)
  fixup_mups6_lo8 = FirstTargetFixupKind,

  // 8-bit fixup corresponding to hi(foo)
  fixup_mups6_hi8,

  // Should have pc-relative fixups for branches and jumps here, too

  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};
} // end namespace Mups16
} // end namespace llvm

#endif
