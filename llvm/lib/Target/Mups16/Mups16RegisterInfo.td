//===-- Mups16RegisterInfo.td - Mups16 Register defs ---------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the Mups16 register file
//===----------------------------------------------------------------------===//

class Mups16Reg<bits<3> num, string n> : Register<n> {
  field bits<3> Num = num;
  let Namespace = "MUPS";
}

// Registers are identified with 5-bit ID numbers.
// Ri - 16-bit integer registers
class Ri<bits<3> Enc, string n> : Mups16Reg<Enc, n>;

// Integer registers
def Zero : Ri< 0, "zero">, DwarfRegNum<[0]>;
def R1 : Ri< 1, "r1">, DwarfRegNum<[1]>;
def R2 : Ri< 2, "r2">, DwarfRegNum<[2]>;
def R3 : Ri< 3, "r3">, DwarfRegNum<[3]>;
def R4 : Ri< 4, "r4">, DwarfRegNum<[4]>;
def SP : Ri< 5, "sp">, DwarfRegNum<[5]>;
def PC : Ri< 6, "pc">, DwarfRegNum<[6]>;
def RA : Ri< 7, "ra">, DwarfRegNum<[7]>;

// System registers (not generally visible to the user, but some instructions use them)
def SPC : Ri< 0, "spc">;
def S1  : Ri< 1, "s1">;
def S2  : Ri< 2, "s2">;
def S3  : Ri< 3, "s3">;
def TSP : Ri< 4, "tsp">;
def TMP : Ri< 5, "tmp">;

// Register classes.
//
// FIXME: the register order should be defined in terms of the preferred
// allocation order...
//
def IntRegs : RegisterClass<"MUPS", [i16], 16, (add Zero, R1, R2, R3, R4, SP, PC, RA)>;
def IntReg : RegisterOperand<IntRegs>;

let isAllocatable = 0 in {
  def SysRegs : RegisterClass<"MUPS", [i16], 16, (add Zero, SP, PC, RA, SPC, S1, S2, S3, TSP, TMP)>;
}
def SysReg : RegisterOperand<SysRegs>;

