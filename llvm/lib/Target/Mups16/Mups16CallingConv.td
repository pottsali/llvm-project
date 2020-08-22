//===-- SparcCallingConv.td - Calling Conventions Sparc ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the Sparc architectures.
//
//===----------------------------------------------------------------------===//

def CC_Mups16 : CallingConv<[
  CCIfType<[i16], CCAssignToReg<[R0, R1]>>,

  // Alternatively, they are assigned to the stack in 4-byte aligned units.
  CCAssignToStack<2, 2>
]>;

def RetCC_Mups16 : CallingConv<[
  CCIfType<[i16], CCAssignToReg<[R0, R1]>>,
  CCAssignToStack<2, 2>
]>;

// Callee-saved registers are handled by the register window mechanism.
def CSR : CalleeSavedRegs<(add)> {
  let OtherPreserved = (add (sequence "R%u", 0, 4));
}
