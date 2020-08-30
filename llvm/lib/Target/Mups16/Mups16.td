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
include "Mups16CallingConv.td"
//include "Mups16RegisterBanks.td"

// Avoid forward declaration issues.
//include "Mups16ScheduleGeneric.td"

def Mups16InstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// Mups16 processors supported.
//===----------------------------------------------------------------------===//
class Proc<string Name, list<SubtargetFeature> Features>
 : Processor<Name, NoItineraries, Features>;

def : Proc<"generic",         []>;
def : Proc<"mups16",          []>;

//===---------------------------------------------------------------------===//
// Assembly Parsers
//===---------------------------------------------------------------------===//

def Mups16AsmParser : AsmParser {
  let ShouldEmitMatchRegisterName = 1;
}

//===----------------------------------------------------------------------===//
// Target Declaration
//===----------------------------------------------------------------------===//

def Mups16 : Target {
  let InstructionSet = Mups16InstrInfo;
  let AssemblyParsers = [Mups16AsmParser];
  let AllowRegisterRenaming = 1;
}
