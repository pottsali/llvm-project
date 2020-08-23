//===-- Mups16MCAsmInfo.cpp - Mups16 asm properties -----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the Mups16MCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "Mups16MCAsmInfo.h"
using namespace llvm;

void Mups16MCAsmInfo::anchor() { }

Mups16MCAsmInfo::Mups16MCAsmInfo(const Triple &TT,
                                 const MCTargetOptions &Options) {
  CodePointerSize = CalleeSaveStackSlotSize = 2;

  CommentString = ";";
  SeparatorString = "{";
}
