//===-- Mups16CallingConv.td - Calling Conventions Sparc ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the Mups16 architectures.
//
//===----------------------------------------------------------------------===//

def CC_Mups16 : CallingConv<[
  // Promote i8 arguments to i16.
  CCIfType<[i8], CCPromoteToType<i16>>,

  // Assign the first two arguments to R1, R2
  CCIfType<[i16], CCAssignToReg<[R1, R2]>>,

  // Alternatively, they are assigned to the stack in 4-byte aligned units.
  CCAssignToStack<2, 2>
]>;

// Return value convention: return value is in R1, with any extra spilled to the stack
def RetCC_Mups16 : CallingConv<[
  CCIfType<[i16], CCAssignToReg<[R1]>>,
  CCAssignToStack<2, 2>
]>;

