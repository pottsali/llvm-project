#ifndef LLVM_LIB_TARGET_MUPS16_MUPS16_H
#define LLVM_LIB_TARGET_MUPS16_MUPS16_H

#include "MCTargetDesc/Mups16MCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm
{
    class Mups16TargetMachine;
    class FunctionPass;

    FunctionPass *createMups16ISelDag(Mups16TargetMachine &TM,
            CodeGenOpt::Level OptLevel);
}

#endif
