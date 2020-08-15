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
def R0 : Ri< 0, "R0">, DwarfRegNum<[0]>;
def R1 : Ri< 1, "R1">, DwarfRegNum<[1]>;
def R2 : Ri< 2, "R2">, DwarfRegNum<[2]>;
def R3 : Ri< 3, "R3">, DwarfRegNum<[3]>;
def R4 : Ri< 4, "R4">, DwarfRegNum<[4]>;
def SP : Ri< 5, "SP">, DwarfRegNum<[5]>;
def PC : Ri< 6, "PC">, DwarfRegNum<[6]>;
def RA : Ri< 7, "RA">, DwarfRegNum<[7]>;

// System registers (not generally visible to the user, but some instructions use them)
def SPC : Ri< 0, "SPC">;
def S1  : Ri< 1, "S1">;
def S2  : Ri< 2, "S2">;
def S3  : Ri< 3, "S3">;
def TSP : Ri< 4, "TSP">;
def TMP : Ri< 5, "TMP">;

// Register classes.
//
// FIXME: the register order should be defined in terms of the preferred
// allocation order...
//
def IntRegs : RegisterClass<"MUPS", [i16], 16, (add R0, R1, R2, R3, R4, SP, PC, RA)>;

let isAllocatable = 0 in {
  def SysRegs : RegisterClass<"MUPS", [i16], 16, (add R0, SP, PC, RA, SPC, S1, S2, S3, TSP, TMP)>;
}

