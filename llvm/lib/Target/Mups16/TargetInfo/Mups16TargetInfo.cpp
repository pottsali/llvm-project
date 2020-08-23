//===-- Mups16TargetInfo.cpp - Mups16 Target Implementation -----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/Mups16TargetInfo.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target &llvm::getTheMups16Target() {
  static Target TheMups16Target;
  return TheMups16Target;
}

extern "C" void LLVMInitializeMups16TargetInfo() {
  RegisterTarget<Triple::mups16> X(getTheMups16Target(), /* name */ "mups16", /* desc */ "Mups16", /* backend name */ "Mups16");
}
