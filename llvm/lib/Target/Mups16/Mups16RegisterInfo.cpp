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
//#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define GET_REGINFO_TARGET_DESC
#include "Mups16GenRegisterInfo.inc"

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

    reserved.set(MUPS::R1);

    return reserved;
}

const TargetRegisterClass* Mups16RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                      unsigned Kind) const
{
    return &MUPS::IntRegsRegClass;
}

void Mups16RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                       int SPAdj, unsigned FIOperandNum,
                                       RegScavenger *RS) const
{
}

Register Mups16RegisterInfo::getFrameRegister(const MachineFunction &MF) const
{
  return MUPS::R2;
}

bool Mups16RegisterInfo::canRealignStack(const MachineFunction &MF) const
{
    // No idea if this is correct
    return false;
}
