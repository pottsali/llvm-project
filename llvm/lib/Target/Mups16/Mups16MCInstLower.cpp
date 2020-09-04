//===-- Mups16MCInstLower.cpp - Convert Mups16 MachineInstr to an MCInst --===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower Mups16 MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "Mups16MCInstLower.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;

MCSymbol *Mups16MCInstLower::GetGlobalAddressSymbol(const MachineOperand &MO) const {
  if (MO.getTargetFlags()) {
      llvm_unreachable("Unknown target flag on GV operand");
  }

  return Printer.getSymbol(MO.getGlobal());
}

MCSymbol *Mups16MCInstLower::GetExternalSymbolSymbol(const MachineOperand &MO) const {
  if (MO.getTargetFlags()) {
      llvm_unreachable("Unknown target flag on GV operand");
  }

  return Printer.GetExternalSymbolSymbol(MO.getSymbolName());
}

MCSymbol *Mups16MCInstLower::GetJumpTableSymbol(const MachineOperand &MO) const {
  if (MO.getTargetFlags()) {
      llvm_unreachable("Unknown target flag on GV operand");
  }

  const DataLayout &DL = Printer.getDataLayout();
  SmallString<256> Name;
  raw_svector_ostream(Name) << DL.getPrivateGlobalPrefix() << "JTI"
                            << Printer.getFunctionNumber() << '_'
                            << MO.getIndex();

  // Create a symbol for the name.
  return Ctx.getOrCreateSymbol(Name);
}

MCSymbol *Mups16MCInstLower::GetConstantPoolIndexSymbol(const MachineOperand &MO) const {
  if (MO.getTargetFlags()) {
      llvm_unreachable("Unknown target flag on GV operand");
  }

  const DataLayout &DL = Printer.getDataLayout();
  SmallString<256> Name;
  raw_svector_ostream(Name) << DL.getPrivateGlobalPrefix() << "CPI"
                            << Printer.getFunctionNumber() << '_'
                            << MO.getIndex();

  // Create a symbol for the name.
  return Ctx.getOrCreateSymbol(Name);
}

MCSymbol *Mups16MCInstLower::GetBlockAddressSymbol(const MachineOperand &MO) const {
  if (MO.getTargetFlags()) {
      llvm_unreachable("Unknown target flag on GV operand");
  }

  return Printer.GetBlockAddressSymbol(MO.getBlockAddress());
}

MCOperand Mups16MCInstLower::LowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym) const {
  if (MO.getTargetFlags()) {
      llvm_unreachable("Unknown target flag on GV operand");
  }

  // FIXME: We would like an efficient form for this, so we don't have to do a
  // lot of extra uniquing.
  const MCExpr *Expr = MCSymbolRefExpr::create(Sym, Ctx);

  if (!MO.isJTI() && MO.getOffset()) {
        Expr = MCBinaryExpr::createAdd(Expr,
                                    MCConstantExpr::create(MO.getOffset(), Ctx),
                                    Ctx);
  }
  return MCOperand::createExpr(Expr);
}

#define GET_REGINFO_ENUM
#include "Mups16GenRegisterInfo.inc"

void Mups16MCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);

    MCOperand MCOp;
    switch (MO.getType()) {
    default:
      MI->print(errs());
      llvm_unreachable("unknown operand type");
    case MachineOperand::MO_Register:
      // Ignore all implicit register operands.
      if (MO.isImplicit()) continue;
      MCOp = MCOperand::createReg(MO.getReg());
      break;
    case MachineOperand::MO_Immediate:
      MCOp = MCOperand::createImm(MO.getImm());
      break;
    case MachineOperand::MO_MachineBasicBlock:
      MCOp = MCOperand::createExpr(MCSymbolRefExpr::create(
                         MO.getMBB()->getSymbol(), Ctx));
      break;
    case MachineOperand::MO_GlobalAddress:
      MCOp = LowerSymbolOperand(MO, GetGlobalAddressSymbol(MO));
      break;
    case MachineOperand::MO_ExternalSymbol:
      MCOp = LowerSymbolOperand(MO, GetExternalSymbolSymbol(MO));
      break;
    case MachineOperand::MO_JumpTableIndex:
      MCOp = LowerSymbolOperand(MO, GetJumpTableSymbol(MO));
      break;
    case MachineOperand::MO_ConstantPoolIndex:
      MCOp = LowerSymbolOperand(MO, GetConstantPoolIndexSymbol(MO));
      break;
    case MachineOperand::MO_BlockAddress:
      MCOp = LowerSymbolOperand(MO, GetBlockAddressSymbol(MO));
      break;
    case MachineOperand::MO_RegisterMask:
      continue;
    }

    OutMI.addOperand(MCOp);
  }
}
