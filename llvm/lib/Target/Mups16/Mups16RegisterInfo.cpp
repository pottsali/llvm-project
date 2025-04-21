//===-- Mups16RegisterInfo.cpp - SPARC Register Information ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the SPARC implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "Mups16.h"
#include "Mups16RegisterInfo.h"
//#include "Mups16MachineFunctionInfo.h"
//#include "Mups16Subtarget.h"
#include "Mups16FrameLowering.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

 #include "llvm/Support/Debug.h"

using namespace llvm;

#define GET_REGINFO_TARGET_DESC
#include "Mups16GenRegisterInfo.inc"

#define DEBUG_TYPE "reginfo"

Mups16RegisterInfo::Mups16RegisterInfo()
    : Mups16GenRegisterInfo(MUPS::RA)
{
}

const MCPhysReg* Mups16RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const
{
    static const MCPhysReg CalleeSavedRegs[] = {
        MUPS::R2, MUPS::R3, MUPS::R4, MUPS::R5, MUPS::RA,
        0
    };
    return CalleeSavedRegs;
}

BitVector Mups16RegisterInfo::getReservedRegs(const MachineFunction &MF) const
{
    BitVector reserved(getNumRegs());

    // FIXME: does this need to contain system registers? Or does the isAllocatable in the register
    // class handle that for us?
    reserved.set(MUPS::R0);
    reserved.set(MUPS::SP);
    reserved.set(MUPS::RA);

    return reserved;
}

const TargetRegisterClass* Mups16RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                      unsigned Kind) const
{
    return &MUPS::IntRegsRegClass;
}

// This is called for every instruction that has an operand that references a frame index. LLVM's
// frame index is an index within an abstract stack. We need to replace these with a direct
// reference to either the stack or frame pointers.
void Mups16RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const
{
    MachineInstr &MI = *II;
    MachineFunction &MF = *MI.getParent()->getParent();
    const TargetInstrInfo *TII = MF.getSubtarget().getInstrInfo();
    const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();

    // Check if we need a frame pointer.
    bool HasFP = TFI->hasFP(MF);
    DebugLoc DL = MI.getDebugLoc();

    // This frame index is the one we stored as the second operand in Mups16InstrInfo::storeRegToStackSlot
    int FrameIndex = MI.getOperand(FIOperandNum).getIndex();

    // In our case the immediate here is always zero. Not sure if could actually just drop it? Some
    // backends use it.
    // In any case, this Offset is the actual byte offset from the beginning of the stack frame.
    int Offset = MF.getFrameInfo().getObjectOffset(FrameIndex) + MI.getOperand(FIOperandNum + 1).getImm();

    // For simplicity for now, just assert that we don't need stack realignment (I don't think we
    // do? Nothing has stricter alighment than the default two bytes), and that we don't need a base
    // pointer (only for complicated cases with dynamic stack allocations, I think?)
    if (needsStackRealignment(MF))
    {
        llvm_unreachable("Stack realignment not supported on Mups16");
    }
    if (hasBasePointer(MF))
    {
        llvm_unreachable("Functions requiring base pointers not supported on Mups16");
    }

    // Stack objects can either be addressed with a negative offset from the frame pointer (R5) or a
    // positive offset from SP. If we have a frame pointer we just use that; in theory, if the
    // offset is more than 15 from the frame pointer we could check if SP is within 16 bytes and use
    // that instead to avoid a LIU/LUI pair to get the offset into a register, but that's an
    // optimisation for later.
    if (!HasFP)
    {
        // Change to positive offset from SP
        Offset += MF.getFrameInfo().getStackSize();
    }

    Register FrameReg = getFrameRegister(MF);

    LLVM_DEBUG(dbgs() << "FrameIndex     : " << FrameIndex << "\n"
                      << "Offset         : " << Offset << "\n"
                      << "FIOperandNum   : " << FIOperandNum << "\n"
                );
    LLVM_DEBUG(dbgs() << "MI before    : " << MI << "\n"
                      << "         op0 : " << MI.getOperand(0) << "\n"
                      << "         op1 : " << MI.getOperand(1) << "\n"
                      << "         op2 : " << MI.getOperand(2) << "\n"
                      );

    // The encoded instruction has a pair of dummy operands, one holding the frame index and one
    // holding a (zero) offset. This code needs to replace the operands with actual ones that we
    // will be able to lower to machine instructions.
    // If the actual Offset value is within the range of a signed 5-bit immediate, we can just
    // replace the frame index with either SP or R5, and the immediate with Offset.
    // If the offset is too big, then we need to first generate some new instructions to load the
    // offset into a (scavenged) register, then change the frame index operand to the new register,
    // and set the offset to zero.
    if (isInt<5>(Offset))
    {
        // Simple case where the offset fits into the immediate.
        MI.getOperand(FIOperandNum).ChangeToRegister(FrameReg, /*isDef=*/false);
        MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
    }
    else
    {
        //assert(RS && "Register scavenging must be on");
        //Register Reg = RS->FindUnusedReg(&MUPS::IntRegsRegClass);
        //assert(Reg && "Register scavenger failed");
        MachineRegisterInfo &MRI = (*MI.getParent()).getParent()->getRegInfo();
        Register Reg = MRI.createVirtualRegister(&MUPS::IntRegsRegClass);

        // Can we load the offset with a single LI call?
        if (isInt<8>(Offset))
        {
            // li $offset_reg, offset
            BuildMI(*MI.getParent(), II, DL, TII->get(MUPS::LI), Reg)
                .addImm(Offset);
        }
        else
        {
            // liu $offset_reg, offset & 0xff
            // lui $offset_reg, offset >> 8
            BuildMI(*MI.getParent(), II, DL, TII->get(MUPS::LIU), Reg)
                .addImm(Offset & 0xffU);
            BuildMI(*MI.getParent(), II, DL, TII->get(MUPS::LUI), Reg)
                .addImm(static_cast<uint32_t>(Offset) >> 8);
        }

        // $offsetreg = $framereg + $offsetreg
        BuildMI(*MI.getParent(), II, DL, TII->get(MUPS::ADD), Reg)
            .addReg(FrameReg)
            .addReg(Reg);

        switch (MI.getOpcode())
        {
        default:
            llvm_unreachable("Unexpected opcode in frame index operation");
        case MUPS::LW:
        case MUPS::LB:
        case MUPS::LBU:
        case MUPS::SW:
        case MUPS::SB:
            break;
        }

        // Reg now has the final address, so change the instruction to be a zero offset from Reg
        MI.getOperand(FIOperandNum).ChangeToRegister(Reg, /*isDef=*/false, false, /*isKill=*/true);
        MI.getOperand(FIOperandNum + 1).ChangeToImmediate(0);
    }
    LLVM_DEBUG(dbgs() << "MI after     : " << MI << "\n"
                      << "         op0 : " << MI.getOperand(0) << "\n"
                      << "         op1 : " << MI.getOperand(1) << "\n"
                      << "         op2 : " << MI.getOperand(2) << "\n"
                      );
}

bool Mups16RegisterInfo::hasBasePointer(const MachineFunction &MF) const
{
    const MachineFrameInfo &MFI = MF.getFrameInfo();
    // When we need stack realignment and there are dynamic allocas, we can't
    // reference off of the stack pointer, so we reserve a base pointer.
    if (needsStackRealignment(MF) && MFI.hasVarSizedObjects())
        llvm_unreachable("functions requiring base pointer not supported");
    return false;
}


Register Mups16RegisterInfo::getFrameRegister(const MachineFunction &MF) const
{
    // If the function needs a frame pointer, use R5. Otherwise, just use SP, and we'll fix up
    // offsets to be positive instead of negative in eliminateFrameIndex above.
    const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
    return TFI->hasFP(MF) ? MUPS::R5 : MUPS::SP;
}

