//===--- Mups16.cpp - Implement Mups16 target feature support -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements Mups16 TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "Mups16.h"
#include "clang/Basic/MacroBuilder.h"

using namespace clang;
using namespace clang::targets;

const char *const Mups16TargetInfo::GCCRegNames[] = {
    "r0", "r1", "r2",  "r3",  "r4",  "r5",  "r6",  "r7"
};

ArrayRef<const char *> Mups16TargetInfo::getGCCRegNames() const {
  return llvm::makeArrayRef(GCCRegNames);
}

void Mups16TargetInfo::getTargetDefines(const LangOptions &Opts,
                                        MacroBuilder &Builder) const {
  Builder.defineMacro("Mups16");
  Builder.defineMacro("__Mups16__");
}
