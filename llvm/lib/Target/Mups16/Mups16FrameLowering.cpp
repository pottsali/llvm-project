//===-- Mups16FrameLowering.cpp - Mups16 Frame Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Mups16 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "Mups16FrameLowering.h"
#include "Mups16Subtarget.h"
#include "Mups16RegisterInfo.h"
#include "Mups16InstrInfo.h"
#include "llvm/CodeGen/MachineFrameInfo.h"

using namespace llvm;

// Determines the size of the frame and maximum call frame size.
void Mups16FrameLowering::determineFrameLayout(MachineFunction &MF) const
{
    MachineFrameInfo &MFI = MF.getFrameInfo();
    const Mups16RegisterInfo *RI = STI.getRegisterInfo();

    // Get the number of bytes to allocate from the FrameInfo.
    unsigned FrameSize = MFI.getStackSize();

    // Get the alignment.
    Align StackAlign = RI->needsStackRealignment(MF) ? MFI.getMaxAlign() : getStackAlign();

    // Get the maximum call frame size of all the calls.
    unsigned MaxCallFrameSize = MFI.getMaxCallFrameSize();

    // If we have dynamic alloca then MaxCallFrameSize needs to be aligned so
    // that allocations will be aligned.
    if (MFI.hasVarSizedObjects())
    {
        MaxCallFrameSize = alignTo(MaxCallFrameSize, StackAlign);
    }

    // Update maximum call frame size.
    MFI.setMaxCallFrameSize(MaxCallFrameSize);

    // Include call frame size in total.
    if (!(hasReservedCallFrame(MF) && MFI.adjustsStack()))
    {
        FrameSize += MaxCallFrameSize;
    }

    // Make sure the frame is aligned.
    FrameSize = alignTo(FrameSize, StackAlign);

    // Update frame info.
    MFI.setStackSize(FrameSize);
}

// Current convention: SP always points to the last element on the stack. 
// On function entry SP will point to the first argument that isn't passed in registers (currently,
// that would be arguments 5 and later):
//
// sp+2     arg5
// sp       arg4
//
// On entry, we save the old frame pointer, generate a new one pointing to the first word after the
// function arguments, and decrement SP. If sp was 0x100 in the above diagram, our frame would end
// up being:
// 
// 0x102    arg5
// 0x100    arg4
// 0x09e    <old fp>    <- fp points here now
// 0x09c    <local>
// 0x09a    <local>     <- sp points here now
//
//
// Generates the following sequence for function entry:
//   sw -2[$sp], $fp        ; push old FP
//   addi $fp, $sp, -2      ; generate new FP
//   addi $sp, $sp, -4      ; allocate stack space (as needed)
void Mups16FrameLowering::emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const
{
    // FIXME: document what this means?
    assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");

    MachineFrameInfo &MFI = MF.getFrameInfo();
    const Mups16InstrInfo &LII =
        *static_cast<const Mups16InstrInfo *>(STI.getInstrInfo());
    MachineBasicBlock::iterator MBBI = MBB.begin();

    // Debug location must be unknown since the first debug location is used
    // to determine the end of the prologue.
    DebugLoc DL;

    // Determine the correct frame layout
    determineFrameLayout(MF);

    // We need an extra 2 bytes on top of whatever the function itself needs to hold the saved frame
    // pointer.
    unsigned StackSize = MFI.getStackSize() + 2;

    // Push old FP
    // sw -2[$sp], $fp
    BuildMI(MBB, MBBI, DL, LII.get(MUPS::SW))
        .addReg(MUPS::SP)
        .addImm(-2)
        .addReg(MUPS::FP)
        .setMIFlag(MachineInstr::FrameSetup);

    // Generate new FP
    // addi $fp, $sp, 4
    BuildMI(MBB, MBBI, DL, LII.get(MUPS::ADDI), MUPS::FP)
        .addReg(MUPS::SP)
        .addImm(-2)
        .setMIFlag(MachineInstr::FrameSetup);

    // Allocate space on the stack. We'll always need at least two bytes, for the saved frame
    // pointer
    // subi $sp, $sp, -4
    BuildMI(MBB, MBBI, DL, LII.get(MUPS::ADDI), MUPS::SP)
        .addReg(MUPS::SP)
        .addImm(-static_cast<int32_t>(StackSize))
        .setMIFlag(MachineInstr::FrameSetup);

    // Replace ADJDYNANALLOC
    // FIXME
    /*
    if (MFI.hasVarSizedObjects())
        replaceAdjDynAllocPseudo(MF);
    */
}

void Mups16FrameLowering::emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const
{
    MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
    const Mups16InstrInfo &LII = *static_cast<const Mups16InstrInfo *>(STI.getInstrInfo());
    DebugLoc DL = MBBI->getDebugLoc();

    // Restore the stack pointer using the callee's frame pointer value.
    BuildMI(MBB, MBBI, DL, LII.get(MUPS::ADDI), MUPS::SP)
        .addReg(MUPS::FP)
        .addImm(0);

    // Restore the frame pointer from the stack.
    BuildMI(MBB, MBBI, DL, LII.get(MUPS::LW), MUPS::FP)
        .addReg(MUPS::FP)
        .addImm(-4);
}


bool Mups16FrameLowering::spillCalleeSavedRegisters(MachineBasicBlock &MBB,
        MachineBasicBlock::iterator MI,
        ArrayRef<CalleeSavedInfo> CSI,
        const TargetRegisterInfo *TRI) const
{
    // FIXME
    return false;
}

bool Mups16FrameLowering::restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
        MachineBasicBlock::iterator MI,
        MutableArrayRef<CalleeSavedInfo> CSI,
        const TargetRegisterInfo *TRI) const
{
    // FIXME
    return false;
}

bool Mups16FrameLowering::hasFP(const MachineFunction &MF) const
{
    return true;
}

