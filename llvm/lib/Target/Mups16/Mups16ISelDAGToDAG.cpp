//===-- Mups16ISelDAGToDAG.cpp - A dag to dag inst selector for Mups16 ----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the Mups16 target.
//
//===----------------------------------------------------------------------===//

#include "Mups16.h"
#include "Mups16TargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetLowering.h"
#include "llvm/Config/llvm-config.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "mups16-isel"

/// Mups16DAGToDAGISel - Mups16 specific code to select Mups16 machine
/// instructions for SelectionDAG operations.
///
namespace
{
    class Mups16DAGToDAGISel : public SelectionDAGISel
    {
    public:
        Mups16DAGToDAGISel(Mups16TargetMachine &TM, CodeGenOpt::Level OptLevel)
            : SelectionDAGISel(TM, OptLevel) {}

    private:
        StringRef getPassName() const override {
            return "Mups16 DAG->DAG Pattern Instruction Selection";
        }

        // Include the pieces autogenerated from the target description.
#include "Mups16GenDAGISel.inc"

        // Main method to transform nodes into machine nodes.
        void Select(SDNode *N) override;
    };
}  // end anonymous namespace

/// createMups16ISelDag - This pass converts a legalized DAG into a
/// Mups16-specific DAG, ready for instruction scheduling.
FunctionPass *llvm::createMups16ISelDag(Mups16TargetMachine &TM, CodeGenOpt::Level OptLevel)
{
    return new Mups16DAGToDAGISel(TM, OptLevel);
}


void Mups16DAGToDAGISel::Select(SDNode *Node)
{
    SDLoc dl(Node);

    // If we have a custom node, we already have selected!
    if (Node->isMachineOpcode()) {
        LLVM_DEBUG(errs() << "== "; Node->dump(CurDAG); errs() << "\n");
        Node->setNodeId(-1);
        return;
    }

    // Select the default instruction
    SelectCode(Node);
}
