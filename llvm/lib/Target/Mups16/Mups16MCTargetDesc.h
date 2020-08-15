
#ifndef LLVM_LIB_TARGET_MUPS16_MUPS16MCTARGETDESC_H
#define LLVM_LIB_TARGET_MUPS16_MUPS16MCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {
class Target;
class Triple;

} // End llvm namespace

// Defines symbolic names for Sparc registers.  This defines a mapping from
// register name to register number.
//
#define GET_REGINFO_ENUM
#include "Mups16GenRegisterInfo.inc"

// Defines symbolic names for the Sparc instructions.
//
#define GET_INSTRINFO_ENUM
#include "Mups16GenInstrInfo.inc"

#endif
