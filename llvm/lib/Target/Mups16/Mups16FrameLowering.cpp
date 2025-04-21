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
#include "Mups16InstrInfo.h"
#include "Mups16RegisterInfo.h"
#include "Mups16Subtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"

 #include "llvm/Support/Debug.h"
#define DEBUG_TYPE "frame"

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

void Mups16FrameLowering::adjustReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                                    const DebugLoc &DL, Register DestReg, Register SrcReg,
                                    int64_t Val, MachineInstr::MIFlag Flag) const
{
    MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
    const Mups16InstrInfo *TII = STI.getInstrInfo();

    LLVM_DEBUG(dbgs() << "Adjusting " << DestReg << " from " << SrcReg << " by " << Val << "\n");
    if (isInt<5>(Val))
    {
        LLVM_DEBUG(dbgs() << " Using addi with 5-bit immediate\n");
        BuildMI(MBB, MBBI, DL, TII->get(MUPS::ADDI), DestReg)
            .addReg(SrcReg)
            .addImm(static_cast<int32_t>(Val))
            .setMIFlag(Flag)
        ;
    }
    else
    {
        Register Reg = MRI.createVirtualRegister(&MUPS::IntRegsRegClass);

        // Can we load the offset with a single LI call?
        if (isInt<8>(Val))
        {
            LLVM_DEBUG(dbgs() << " Loading " << Reg << " with 8-bit immediate\n");
            // li reg, offset
            BuildMI(MBB, MBBI, DL, TII->get(MUPS::LI), Reg)
                .addImm(Val)
                .setMIFlag(Flag)
                ;
        }
        else
        {
            LLVM_DEBUG(dbgs() << " Loading " << Reg << " using liu/lui\n");
            // liu reg, offset & 0xff
            // lui reg, offset >> 8
            BuildMI(MBB, MBBI, DL, TII->get(MUPS::LIU), Reg)
                .addImm(Val & 0xffU)
                .setMIFlag(Flag)
                ;
            BuildMI(MBB, MBBI, DL, TII->get(MUPS::LUI), Reg)
                .addImm(static_cast<uint32_t>(Val) >> 8)
                .setMIFlag(Flag)
                ;
        }

        LLVM_DEBUG(dbgs() << " add " << DestReg << ", " << SrcReg << ", " << Reg << "\n");
        BuildMI(MBB, MBBI, DL, TII->get(MUPS::ADD), DestReg)
            .addReg(SrcReg)
            .addReg(Reg, RegState::Kill)
            .setMIFlag(Flag)
            ;
    }
}

// Current convention: SP always points to the last element on the stack. On function entry SP will
// point to the second function argument, since only the first is passed in a register (R1). All
// other args are passed on the stack:
//
// sp+2     arg2
// sp       arg1
//
// If the function needs a frame pointer then we use R5. This will point to the stack slot holding
// the saved value of R5 (i.e. 2 below the incoming SP). This seems to be what LLVM expects by
// default, and means that the computed stack offsets just work. Ideally, we'd be able to point R5
// one slot below that (to the first local on the stack) to reduce the distance from R5 to the
// locals on the stack by 2, which increases the chances that variables on the stack will fall with
// a 5-bit immediate offset of R5.
//
// On entry, we save the old frame pointer, generate a new one pointing to the first word after the
// function arguments, and decrement SP by whatever's required for the function.
// If incoming sp was 0x100 in the above case (with two args, and, say, three words of local stack
// usage), our frame would end up being:
//
// 0x100    arg1
// 0x09e    <old r5>    <- r5 points here now
// 0x09c    <local>
// 0x09a    <local>
// 0x098    <local>     <- sp points here now
//
// and would generate the following sequence for function entry:
//   sw (-2)sp, r5        ; push old FP
//   addi r5, sp, -2      ; generate new FP pointing one above first stack slot
//   addi sp, sp, xx      ; allocate stack space (as needed)
void Mups16FrameLowering::emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const
{
    // Shrink-wrapping attempts to move frame setup to somewhere other than the
    // first instruction of the function (this is useful if we only use the
    // stack in one branch of an if, for example). We don't support that
    // (yet?).
    // See https://reviews.llvm.org/D9210
    assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");

    MachineFrameInfo &MFI = MF.getFrameInfo();
    MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
    const Mups16InstrInfo &LII = *static_cast<const Mups16InstrInfo *>(STI.getInstrInfo());
    MachineBasicBlock::iterator MBBI = MBB.begin();

    // Debug location must be unknown since the first debug location is used
    // to determine the end of the prologue.
    DebugLoc DL;

    // Determine the correct frame layout
    determineFrameLayout(MF);

    // As a trivial optimisation, if the function doesn't use any stack itself (either for storage,
    // or calling other functions), then don't bother saving the frame pointer or decrementing SP.
    if (MFI.getStackSize() == 0 && !MFI.adjustsStack())
    {
        return;
    }

    int StackSize = MFI.getStackSize();

    // If we're using a frame pointer, then we need to save the old value of r5, so add 2 to the
    // needed stack size.
    if (hasFP(MF))
    {
        // We need an extra 2 bytes on top of whatever the function itself needs to hold the saved frame
        // pointer.
        StackSize += 2;

        LLVM_DEBUG(dbgs() << "Function needs frame pointer, using stack size of " << StackSize << "\n");

        // Push old r5
        // sw (-2)sp, r5
        BuildMI(MBB, MBBI, DL, LII.get(MUPS::SW))
            .addReg(MUPS::SP)
            .addImm(-2)
            .addReg(MUPS::R5)
            .setMIFlag(MachineInstr::FrameSetup);

        // addi r5, sp, -4
        BuildMI(MBB, MBBI, DL, LII.get(MUPS::ADDI), MUPS::R5)
            .addReg(MUPS::SP)
            .addImm(-2)
            .setMIFlag(MachineInstr::FrameSetup);
    }

    // Adjust SP. If the stack size is greater than 15 (max negative range of addi) then we need to use a
    // liu/lui/add triple to get the value into sp.
    adjustReg(MBB, MBBI, DL, MUPS::SP, MUPS::SP, -StackSize, MachineInstr::FrameSetup);

    // Replace ADJDYNANALLOC
    // FIXME
    /*
    if (MFI.hasVarSizedObjects())
        replaceAdjDynAllocPseudo(MF);
    */
}

void Mups16FrameLowering::emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const
{
    MachineFrameInfo &MFI = MF.getFrameInfo();
    MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
    const Mups16InstrInfo &LII = *static_cast<const Mups16InstrInfo *>(STI.getInstrInfo());
    DebugLoc DL = MBBI->getDebugLoc();

    // Similar to emitPrologue, if the function didn't use the stack at all,
    // skip restoring the frame pointer and incrementing SP.
    if (MFI.getStackSize() == 0 && !MFI.adjustsStack())
    {
        return;
    }

    unsigned StackSize = MFI.getStackSize();

    if (hasFP(MF))
    {
        StackSize += 2;
        // Restore the stack pointer using r5.
        // addi sp, r5, 4
        BuildMI(MBB, MBBI, DL, LII.get(MUPS::ADDI), MUPS::SP)
            .addReg(MUPS::R5)
            .addImm(2);

        // Restore the saved r5 register from the stack. Note this will be -2 from the SP we
        // restored above
        BuildMI(MBB, MBBI, DL, LII.get(MUPS::LW), MUPS::R5)
            .addReg(MUPS::SP)
            .addImm(-2);
    }
    else
    {
        adjustReg(MBB, MBBI, DL, MUPS::SP, MUPS::SP, StackSize, MachineInstr::FrameDestroy);
    }
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
    // Ideally, we wouldn't need a frame pointer, but some circumstances require us to burn one
    const MachineFrameInfo &MFI = MF.getFrameInfo();

    return (MF.getTarget().Options.DisableFramePointerElim(MF) ||
        MF.getFrameInfo().hasVarSizedObjects() ||
        MFI.isFrameAddressTaken())
    ;
}

void Mups16FrameLowering::determineCalleeSaves(MachineFunction &MF,
        BitVector &SavedRegs, RegScavenger *RS) const
{
    TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

    // If we are going to need to spill a register to get a scratch one (for large immediate loads)
    // in our frame index elimination, add a stack slot to hold the spilled register.
    if (RS)
    {
        // Allocate a spill slot (size, alignment, not in CSR)
        int FI = MF.getFrameInfo().CreateStackObject(2, Align(2), false);
        RS->addScavengingFrameIndex(FI);
    }
}
