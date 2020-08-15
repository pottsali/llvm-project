//===-- Mups16.td - Describe the Mups/16 Target Machine ---------*- tablegen -*-===//
//
//===----------------------------------------------------------------------===//
// This is the top level entry point for the Mups/16 target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "Mups16RegisterInfo.td"
//include "Mups16Schedule.td"
include "Mups16InstrInfo.td"
//include "Mups16CallingConv.td"
//include "Mups16RegisterBanks.td"

// Avoid forward declaration issues.
//include "Mups16ScheduleGeneric.td"

def Mups16InstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// Mups16 processors supported.
//===----------------------------------------------------------------------===//

def Mups16AsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 0;
}

def Mups16AsmParserVariant : AsmParserVariant {
  int Variant = 0;

  // Recognize hard coded registers.
  string RegisterPrefix = "$";
}

def Mups16 : Target {
  let InstructionSet = Mups16InstrInfo;
  let AssemblyParsers = [Mups16AsmParser];
  let AssemblyParserVariants = [Mups16AsmParserVariant];
  let AllowRegisterRenaming = 1;
}
