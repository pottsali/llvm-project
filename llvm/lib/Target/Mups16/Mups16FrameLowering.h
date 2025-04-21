//===-- Mups16FrameLowering.h - Define frame lowering for Mups16 ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MUPS16_MUPS16FRAMELOWERING_H
#define LLVM_LIB_TARGET_MUPS16_MUPS16FRAMELOWERING_H

#include "Mups16.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {
  class Mups16Subtarget;

class Mups16FrameLowering : public TargetFrameLowering
{
public:
    explicit Mups16FrameLowering(const Mups16Subtarget& subtarget)
        : TargetFrameLowering(StackGrowsDown, Align(2), 0, Align(2)),
          STI(subtarget)
    {
    }

    /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
    /// the function.
    void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
    void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;

    bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator MI,
                                    ArrayRef<CalleeSavedInfo> CSI,
                                    const TargetRegisterInfo *TRI) const override;
    bool restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                MutableArrayRef<CalleeSavedInfo> CSI,
                                const TargetRegisterInfo *TRI) const override;

    bool hasFP(const MachineFunction &MF) const override;

    // Overridden so that we can set up a stack slot for emergency spills, needed if we have to
    // scavenge a register to emit a LI or LUI/LIU pair for large immediates.
    void determineCalleeSaves(MachineFunction &MF, BitVector &SavedRegs,
                              RegScavenger *RS = nullptr) const override;


private:
    // Helper
    void determineFrameLayout(MachineFunction &MF) const;

    // Helper that inserts the minimal set of instructions to adjust a register by an immediate
    void adjustReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                    const DebugLoc &DL, Register DestReg, Register SrcReg,
                    int64_t Val, MachineInstr::MIFlag Flag) const;

    const Mups16Subtarget &STI;
};

} // End llvm namespace

#endif

