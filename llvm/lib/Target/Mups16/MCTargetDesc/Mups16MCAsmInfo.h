//===-- Mups16MCAsmInfo.h - Mups16 asm properties --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the Mups16MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16MCASMINFO_H
#define LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16MCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class Mups16MCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit Mups16MCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
};

} // namespace llvm

#endif
