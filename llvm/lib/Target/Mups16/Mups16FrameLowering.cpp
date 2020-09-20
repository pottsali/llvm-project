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

using namespace llvm;


void Mups16FrameLowering::emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const
{
    // FIXME
    report_fatal_error("emitPrologue");
}

void Mups16FrameLowering::emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const
{
    // FIXME
    report_fatal_error("emitEpilogue");
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
    return false;
}

