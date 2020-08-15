//===-- Mups16TargetMachine.cpp - Define TargetMachine for Mups16 -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#include "Mups16TargetMachine.h"
#include "LeonPasses.h"
#include "Mups16.h"
#include "Mups16TargetObjectFile.h"
#include "TargetInfo/Mups16TargetInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

extern "C" void LLVMInitializeMups16Target() {
  // Register the target.
  RegisterTargetMachine<Mups16V8TargetMachine> X(getTheMups16Target());
  RegisterTargetMachine<Mups16V9TargetMachine> Y(getTheMups16V9Target());
  RegisterTargetMachine<Mups16elTargetMachine> Z(getTheMups16elTarget());
}

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

/// Create an ILP32 architecture model
Mups16TargetMachine::Mups16TargetMachine(const Target &T, const Triple &TT, StringRef CPU, StringRef FS,
    const TargetOptions &Options, Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
    CodeGenOpt::Level OL, bool JIT, bool is64bit)
  : LLVMTargetMachine(T, "E-i16:16-p16:16", TT, CPU, FS, Options,
                        getEffectiveRelocModel(RM), CodeModel::Small, OL),
      TLOF(std::make_unique<Mups16ELFTargetObjectFile>()),
      Subtarget(TT, CPU, FS, *this, is64bit), is64Bit(is64bit) 
{
    initAsmInfo();
}

Mups16TargetMachine::~Mups16TargetMachine() {}

namespace {
/// Mups16 Code Generator Pass Configuration Options.
class Mups16PassConfig : public TargetPassConfig {
public:
    Mups16PassConfig(Mups16TargetMachine &TM, PassManagerBase &PM)
        : TargetPassConfig(TM, PM) {}

    Mups16TargetMachine &getMups16TargetMachine() const
    {
        return getTM<Mups16TargetMachine>();
    }

    void addIRPasses() override;
    bool addInstSelector() override;
    void addPreEmitPass() override;
};
} // namespace

TargetPassConfig *Mups16TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new Mups16PassConfig(*this, PM);
}

void Mups16PassConfig::addIRPasses()
{
    addPass(createAtomicExpandPass());
    TargetPassConfig::addIRPasses();
}

bool Mups16PassConfig::addInstSelector()
{
    addPass(createMups16ISelDag(getMups16TargetMachine()));
    return false;
}

void Mups16PassConfig::addPreEmitPass()
{
    addPass(new InsertNOPLoad());
    addPass(new FixAllFDIVSQRT());
}

void Mups16TargetMachine::anchor() { }

