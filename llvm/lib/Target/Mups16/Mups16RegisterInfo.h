//===-- Mups16RegisterInfo.h - Mups16 Register Information Impl ---*- C++ -*-===//
//
//===----------------------------------------------------------------------===//
//
// This file contains the Mups16 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MUPS16_REGISTERINFO_H
#define LLVM_LIB_TARGET_MUPS16_REGISTERINFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "Mups16GenRegisterInfo.inc"

namespace llvm {

    struct Mups16RegisterInfo : public Mups16GenRegisterInfo
    {
        Mups16RegisterInfo();

        /// Code Generation virtual methods...
        const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;
        BitVector getReservedRegs(const MachineFunction &MF) const override;

        const TargetRegisterClass *getPointerRegClass(const MachineFunction &MF,
                unsigned Kind) const override;

        void eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj, unsigned FIOperandNum,
                RegScavenger *RS = nullptr) const override;

        Register getFrameRegister(const MachineFunction &MF) const override;

        bool hasBasePointer(const MachineFunction &MF) const;

        // These are required so that we can use extra registers in our frame index elimination,
        // whenever we need to load large immediates. Setting requiresFrameIndexScavenging allows us to
        // use virtual registers in the eliminateFrameIndex function. Note we still can't directly use
        // the register scavenger; to enable that we would have to also override
        // requiresFrameIndexReplacementScavenging
        bool requiresRegisterScavenging(const MachineFunction &MF) const override { return true; }
        bool requiresFrameIndexScavenging(const MachineFunction &MF) const override { return true; }

    };

} // end namespace llvm

#endif
