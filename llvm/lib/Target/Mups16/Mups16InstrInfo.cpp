//===-- Mups16InstrInfo.cpp - Mups16 Instruction Information --------------===//
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

#include "Mups16InstrInfo.h"
#include "Mups16.h"
//#include "Mups16MachineFunctionInfo.h"
#include "Mups16TargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_CTOR_DTOR
#include "Mups16GenInstrInfo.inc"

// Pin the vtable to this file.
void Mups16InstrInfo::anchor() {}

Mups16InstrInfo::Mups16InstrInfo(Mups16Subtarget &STI)
  : Mups16GenInstrInfo(),
    RI()
{
}

void Mups16InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator MI,
                                          Register SrcReg, bool isKill, int FrameIdx,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const
{
    DebugLoc DL;
    if (MI != MBB.end())
        DL = MI->getDebugLoc();
    MachineFunction &MF = *MBB.getParent();
    MachineFrameInfo &MFI = MF.getFrameInfo();

    MachineMemOperand *MMO = MF.getMachineMemOperand(
            MachinePointerInfo::getFixedStack(MF, FrameIdx),
            MachineMemOperand::MOStore, MFI.getObjectSize(FrameIdx),
            MFI.getObjectAlign(FrameIdx));

    if (RC == &MUPS::IntRegsRegClass)
    {
        BuildMI(MBB, MI, DL, get(MUPS::SW))
            .addFrameIndex(FrameIdx).addImm(0)
            .addReg(SrcReg, getKillRegState(isKill)).addMemOperand(MMO);
    }
    else
        llvm_unreachable("Cannot store this register to stack slot!");
}

void Mups16InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator MI,
                                           Register DestReg, int FrameIdx,
                                           const TargetRegisterClass *RC,
                                           const TargetRegisterInfo *TRI) const
{
    DebugLoc DL;
    if (MI != MBB.end())
        DL = MI->getDebugLoc();
    MachineFunction &MF = *MBB.getParent();
    MachineFrameInfo &MFI = MF.getFrameInfo();

    MachineMemOperand *MMO = MF.getMachineMemOperand(
            MachinePointerInfo::getFixedStack(MF, FrameIdx),
            MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIdx),
            MFI.getObjectAlign(FrameIdx));

    if (RC == &MUPS::IntRegsRegClass)
        BuildMI(MBB, MI, DL, get(MUPS::LW))
            .addReg(DestReg, getDefRegState(true)).addFrameIndex(FrameIdx)
            .addImm(0).addMemOperand(MMO);
    else
        llvm_unreachable("Cannot store this register to stack slot!");
}

void Mups16InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I,
                                  const DebugLoc &DL, MCRegister DestReg,
                                  MCRegister SrcReg, bool KillSrc) const
{
    unsigned Opc;
    if (MUPS::IntRegsRegClass.contains(DestReg, SrcReg))
        Opc = MUPS::ADD;
    else
        llvm_unreachable("Impossible reg-to-reg copy");

    BuildMI(MBB, I, DL, get(Opc), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addReg(MUPS::Zero);
}

/*
unsigned Mups16InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                       int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr())
      continue;
    if (I->getOpcode() != MUPS::JMP &&
        I->getOpcode() != MUPS::JCC &&
        I->getOpcode() != MUPS::Br &&
        I->getOpcode() != MUPS::Bm)
      break;
    // Remove the branch.
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

bool Mups16InstrInfo::
reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid Xbranch condition!");

  Mups16CC::CondCodes CC = static_cast<Mups16CC::CondCodes>(Cond[0].getImm());

  switch (CC) {
  default: llvm_unreachable("Invalid branch condition!");
  case Mups16CC::COND_E:
    CC = Mups16CC::COND_NE;
    break;
  case Mups16CC::COND_NE:
    CC = Mups16CC::COND_E;
    break;
  case Mups16CC::COND_L:
    CC = Mups16CC::COND_GE;
    break;
  case Mups16CC::COND_GE:
    CC = Mups16CC::COND_L;
    break;
  case Mups16CC::COND_HS:
    CC = Mups16CC::COND_LO;
    break;
  case Mups16CC::COND_LO:
    CC = Mups16CC::COND_HS;
    break;
  }

  Cond[0].setImm(CC);
  return false;
}

bool Mups16InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *&TBB,
                                    MachineBasicBlock *&FBB,
                                    SmallVectorImpl<MachineOperand> &Cond,
                                    bool AllowModify) const {
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end();
  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr())
      continue;

    // Working from the bottom, when we see a non-terminator
    // instruction, we're done.
    if (!isUnpredicatedTerminator(*I))
      break;

    // A terminator that isn't a branch can't easily be handled
    // by this analysis.
    if (!I->isBranch())
      return true;

    // Cannot handle indirect branches.
    if (I->getOpcode() == MUPS::Br ||
        I->getOpcode() == MUPS::Bm)
      return true;

    // Handle unconditional branches.
    if (I->getOpcode() == MUPS::JMP) {
      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a JMP, delete them.
      while (std::next(I) != MBB.end())
        std::next(I)->eraseFromParent();
      Cond.clear();
      FBB = nullptr;

      // Delete the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = nullptr;
        I->eraseFromParent();
        I = MBB.end();
        continue;
      }

      // TBB is used to indicate the unconditinal destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    assert(I->getOpcode() == MUPS::JCC && "Invalid conditional branch");
    Mups16CC::CondCodes BranchCode =
      static_cast<Mups16CC::CondCodes>(I->getOperand(1).getImm());
    if (BranchCode == Mups16CC::COND_INVALID)
      return true;  // Can't handle weird stuff.

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      FBB = TBB;
      TBB = I->getOperand(0).getMBB();
      Cond.push_back(MachineOperand::CreateImm(BranchCode));
      continue;
    }

    // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination.
    assert(Cond.size() == 1);
    assert(TBB);

    // Only handle the case where all conditional branches branch to
    // the same destination.
    if (TBB != I->getOperand(0).getMBB())
      return true;

    Mups16CC::CondCodes OldBranchCode = (Mups16CC::CondCodes)Cond[0].getImm();
    // If the conditions are the same, we can leave them alone.
    if (OldBranchCode == BranchCode)
      continue;

    return true;
  }

  return false;
}

unsigned Mups16InstrInfo::insertBranch(MachineBasicBlock &MBB,
                                       MachineBasicBlock *TBB,
                                       MachineBasicBlock *FBB,
                                       ArrayRef<MachineOperand> Cond,
                                       const DebugLoc &DL,
                                       int *BytesAdded) const {
  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "Mups16 branch conditions have one component!");
  assert(!BytesAdded && "code size not handled");

  if (Cond.empty()) {
    // Unconditional branch?
    assert(!FBB && "Unconditional branch with multiple successors!");
    BuildMI(&MBB, DL, get(MUPS::JMP)).addMBB(TBB);
    return 1;
  }

  // Conditional branch.
  unsigned Count = 0;
  BuildMI(&MBB, DL, get(MUPS::JCC)).addMBB(TBB).addImm(Cond[0].getImm());
  ++Count;

  if (FBB) {
    // Two-way Conditional branch. Insert the second branch.
    BuildMI(&MBB, DL, get(MUPS::JMP)).addMBB(FBB);
    ++Count;
  }
  return Count;
}
*/
/// GetInstSize - Return the number of bytes of code the specified
/// instruction may be.  This returns the maximum number of bytes.
///
unsigned Mups16InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const
{
    const MCInstrDesc &Desc = MI.getDesc();

    switch (Desc.getOpcode()) {
        case TargetOpcode::CFI_INSTRUCTION:
        case TargetOpcode::EH_LABEL:
        case TargetOpcode::IMPLICIT_DEF:
        case TargetOpcode::KILL:
        case TargetOpcode::DBG_VALUE:
            return 0;
        default:
            break;
    }

    return Desc.getSize();
}

