#ifndef LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16BASEINFO_H
#define LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16BASEINFO_H

namespace llvm {

namespace Mups16 {
  /// Target Operand Flag enum.
  enum TOF {
    /// MO_ABS_HI/LO - Represents the hi or low part of an absolute symbol
    /// address.
    MO_ABS_HI,
    MO_ABS_LO
  };
} // namespace Mups16

} // namespace llvm

#endif
