//===-- Mups16InstPrinter.cpp - Convert Mups16 MCInst to assembly syntax --===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an Mups16 MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "Mups16InstPrinter.h"
#include "Mups16.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "Mups16GenAsmWriter.inc"

void Mups16InstPrinter::printInst(const MCInst *MI, uint64_t Address,
                                  StringRef Annot, const MCSubtargetInfo &STI,
                                  raw_ostream &O) {
  if (!printAliasInstr(MI, Address, O))
    printInstruction(MI, Address, O);
  printAnnotation(O, Annot);
}

void Mups16InstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                     raw_ostream &O, const char *Modifier) {
  assert((Modifier == nullptr || Modifier[0] == 0) && "No modifiers supported");
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    O << getRegisterName(Op.getReg());
  } else if (Op.isImm()) {
    O << '#' << Op.getImm();
  } else {
    assert(Op.isExpr() && "unknown operand kind in printOperand");
    O << '#';
    Op.getExpr()->print(O, &MAI);
  }
}

template <unsigned Bits>
void Mups16InstPrinter::printUImm(const MCInst *MI, int opNum, raw_ostream &O) {
  const MCOperand &MO = MI->getOperand(opNum);
  if (MO.isImm()) {
    uint64_t Imm = MO.getImm();
    Imm &= (1 << Bits) - 1;
    O << formatImm(Imm);
    return;
  }
  printOperand(MI, opNum, O);
}
