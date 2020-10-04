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

#define DEBUG_TYPE "flailing-around"

Mups16RegisterInfo::Mups16RegisterInfo()
    : Mups16GenRegisterInfo(MUPS::RA)
{
}

const MCPhysReg* Mups16RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const
{
    static const MCPhysReg CalleeSavedRegs[] = {
        MUPS::R1, MUPS::R2, MUPS::R3, MUPS::R4, MUPS::RA,
        0
    };
    return CalleeSavedRegs;
}

BitVector Mups16RegisterInfo::getReservedRegs(const MachineFunction &MF) const
{
    BitVector reserved(getNumRegs());

    // FIXME: this should include all reserved regs, I think (Zero, RA, SP, system regs)

    //reserved.set(MUPS::R1);

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

    // If possible, keep this code working even if we don't have a frame pointer.
    bool HasFP = TFI->hasFP(MF);
    DebugLoc DL = MI.getDebugLoc();

    int FrameIndex = MI.getOperand(FIOperandNum).getIndex();

    int Offset = MF.getFrameInfo().getObjectOffset(FrameIndex) + MI.getOperand(FIOperandNum + 1).getImm();

    // Addressable stack objects are addressed using neg. offsets from fp
    // or pos. offsets from sp/basepointer
    if (!HasFP || (needsStackRealignment(MF) && FrameIndex >= 0))
        Offset += MF.getFrameInfo().getStackSize();

    Register FrameReg = getFrameRegister(MF);
    if (FrameIndex >= 0)
    {
        if (hasBasePointer(MF))
            FrameReg = MUPS::FP;
        else if (needsStackRealignment(MF))
            FrameReg = MUPS::SP;
    }

    LLVM_DEBUG(dbgs() << "FrameIndex     : " << FrameIndex << "\n"
                      << "Offset         : " << Offset << "\n"
                      << "FIOperandNum   : " << FIOperandNum << "\n"
                );

    // Replace frame index with a frame pointer reference.
    // If the offset is small enough to fit in the immediate field, directly
    // encode it.
    // Otherwise scavenge a register and encode it into a LIU, LUI sequence.
    // The only instructions that can have a frame pointer arg with the addressing modes Mups16
    // supports are load/store ones, which have a 5-bit immediate.
    if (!isInt<5>(Offset))
    {
        assert(RS && "Register scavenging must be on");
        unsigned Reg = RS->FindUnusedReg(&MUPS::IntRegsRegClass);
        assert(Reg && "Register scavenger failed");

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

        // reg = $framereg + $offsetreg
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
    else
    {
        // Simple case where the offset fits into the immediate.
        MI.getOperand(FIOperandNum).ChangeToRegister(FrameReg, /*isDef=*/false);
        MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
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
        return true;
    return false;
}


Register Mups16RegisterInfo::getFrameRegister(const MachineFunction &MF) const
{
    return MUPS::FP;
}

