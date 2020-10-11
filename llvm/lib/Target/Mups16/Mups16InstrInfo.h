//===-- Mups16InstrInfo.h - Mups16 Instruction Information ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Mups16 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MUPS16_MUPS16INSTRINFO_H
#define LLVM_LIB_TARGET_MUPS16_MUPS16INSTRINFO_H

#include "Mups16RegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "Mups16GenInstrInfo.inc"

namespace llvm {

class Mups16Subtarget;

class Mups16InstrInfo : public Mups16GenInstrInfo
{
    const Mups16RegisterInfo RI;
    virtual void anchor();
public:
    explicit Mups16InstrInfo(Mups16Subtarget &STI);


    unsigned isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const override;
    unsigned isStoreToStackSlot(const MachineInstr &MI, int &FrameIndex) const override;


    /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
    /// such, whenever a client has an instance of instruction info, it should
    /// always be able to get register info as well (through this method).
    ///
    const Mups16RegisterInfo &getRegisterInfo() const { return RI; }

    void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
            const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg,
            bool KillSrc) const override;

    void storeRegToStackSlot(MachineBasicBlock &MBB,
            MachineBasicBlock::iterator MI,
            Register SrcReg, bool isKill,
            int FrameIndex,
            const TargetRegisterClass *RC,
            const TargetRegisterInfo *TRI) const override;
    void loadRegFromStackSlot(MachineBasicBlock &MBB,
            MachineBasicBlock::iterator MI,
            Register DestReg, int FrameIdx,
            const TargetRegisterClass *RC,
            const TargetRegisterInfo *TRI) const override;

    unsigned getInstSizeInBytes(const MachineInstr &MI) const override;

    /*
    // Branch folding goodness
    bool
    reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const override;
    bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                        MachineBasicBlock *&FBB,
                        SmallVectorImpl<MachineOperand> &Cond,
                        bool AllowModify) const override;

    unsigned removeBranch(MachineBasicBlock &MBB,
                            int *BytesRemoved = nullptr) const override;
    unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                            MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                            const DebugLoc &DL,
                            int *BytesAdded = nullptr) const override;

    int64_t getFramePoppedByCallee(const MachineInstr &I) const {
        assert(isFrameInstr(I) && "Not a frame instruction");
        assert(I.getOperand(1).getImm() >= 0 && "Size must not be negative");
        return I.getOperand(1).getImm();
    }
    */

    // Expand psuedo-instructions like ret into real ones
    bool expandPostRAPseudo(MachineInstr &MI) const override;

private:
    // Expand ret into 'jr ra'
    void expandRetRA(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;

    void expandLoadImm(MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const;

};

}

#endif

