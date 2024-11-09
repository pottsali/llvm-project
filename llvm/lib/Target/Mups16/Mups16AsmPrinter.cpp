//===-- Mups16AsmPrinter.cpp - Mups16 LLVM assembly writer ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to the Mups16 assembly language.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Mups16InstPrinter.h"
#include "MCTargetDesc/Mups16MCExpr.h"
#include "Mups16.h"
//#include "Mups16InstrInfo.h"
#include "Mups16MCInstLower.h"
#include "Mups16TargetMachine.h"
#include "TargetInfo/Mups16TargetInfo.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Mangler.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

namespace {
  class Mups16AsmPrinter : public AsmPrinter
  {
  public:
    Mups16AsmPrinter(TargetMachine &TM, std::unique_ptr<MCStreamer> Streamer)
    : AsmPrinter(TM, std::move(Streamer)) {}

    static const char *getRegisterName(unsigned RegNo)
    {
      return Mups16InstPrinter::getRegisterName(RegNo);
    }

    StringRef getPassName() const override { return "Mups16 Assembly Printer"; }

    void printOperand(const MachineInstr *MI, int opNum, raw_ostream &OS);
    void emitInstruction(const MachineInstr *MI) override;
  };
} // end of anonymous namespace
//
namespace llvm {
  void LowerMups16MachineInstrToMCInst(const MachineInstr *MI,
                                      MCInst &OutMI,
                                      AsmPrinter &AP);
}

//===----------------------------------------------------------------------===//
void Mups16AsmPrinter::emitInstruction(const MachineInstr *MI)
{
  Mups16MCInstLower MCInstLowering(OutContext, *this);

  MCInst TmpInst;
  LowerMups16MachineInstrToMCInst(MI, TmpInst, *this);
  EmitToStreamer(*OutStreamer, TmpInst);
}


void Mups16AsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                   raw_ostream &O) {
  const DataLayout &DL = getDataLayout();
  const MachineOperand &MO = MI->getOperand (opNum);
  Mups16MCExpr::VariantKind TF = (Mups16MCExpr::VariantKind) MO.getTargetFlags();
  bool CloseParen = Mups16MCExpr::printVariantKind(O, TF);
  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << StringRef(getRegisterName(MO.getReg())).lower();
    break;

  case MachineOperand::MO_Immediate:
    O << (int)MO.getImm();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MO.getMBB()->getSymbol()->print(O, MAI);
    return;
  case MachineOperand::MO_GlobalAddress:
    PrintSymbolOperand(MO, O);
    break;
  case MachineOperand::MO_BlockAddress:
    O <<  GetBlockAddressSymbol(MO.getBlockAddress())->getName();
    break;
  case MachineOperand::MO_ExternalSymbol:
    O << MO.getSymbolName();
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    O << DL.getPrivateGlobalPrefix() << "CPI" << getFunctionNumber() << "_"
      << MO.getIndex();
    break;
  case MachineOperand::MO_Metadata:
    MO.getMetadata()->printAsOperand(O, MMI->getModule());
    break;
  default:
    llvm_unreachable("<unknown operand type>");
  }
  if (CloseParen) O << ")";
}


// Force static initialization.
extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMups16AsmPrinter() {
  RegisterAsmPrinter<Mups16AsmPrinter> X(getTheMups16Target());
}
