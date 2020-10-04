//===-- Mups16Subtarget.cpp - Mups16 Subtarget Information ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the Mups16 specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "Mups16Subtarget.h"
#include "Mups16.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "mups16-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "Mups16GenSubtargetInfo.inc"

void Mups16Subtarget::anchor()
{ 
}

Mups16Subtarget::Mups16Subtarget(const Triple &TT, const std::string &CPU,
                                 const std::string &FS, const TargetMachine &TM)
    : Mups16GenSubtargetInfo(TT, CPU, FS), FrameLowering(*this),
      InstrInfo(*this), TLInfo(TM, *this)
{
}
