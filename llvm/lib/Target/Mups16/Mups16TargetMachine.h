//===- MipsTargetMachine.h - Define TargetMachine for Mups16 ------*- C++ -*-===//
//
// This file declares the Mups16 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MUPS16_MIPSTARGETMACHINE_H
#define LLVM_LIB_TARGET_MUPS16_MIPSTARGETMACHINE_H

#include "llvm/ADT/Optional.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

namespace llvm
{

class Mups16TargetMachine : public LLVMTargetMachine 
{
    Mups16Subtarget Subtarget;

public:
    MipsTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
            StringRef FS, const TargetOptions &Options,
            Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
            CodeGenOpt::Level OL, bool JIT);
    ~MipsTargetMachine() override;
    void anchor() override;

    TargetTransformInfo getTargetTransformInfo(const Function &F) override;

    const MipsSubtarget *getSubtargetImpl() const { return &Subtarget; }
    const MipsSubtarget *getSubtargetImpl(const Function &F) const override;

    // Pass Pipeline Configuration
    TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

    TargetLoweringObjectFile *getObjFileLowering() const override {
        return TLOF.get();
    }

    const MipsABIInfo &getABI() const { return ABI; }
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MIPS_MIPSTARGETMACHINE_H
