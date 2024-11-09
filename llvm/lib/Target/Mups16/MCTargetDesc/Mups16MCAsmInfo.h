#ifndef LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_SPARCMCASMINFO_H
#define LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_SPARCMCASMINFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {

class Triple;

// NOTE: this derives from MCAsmInfo, not MCAsmInfoELF, since I'm targeting
// customasm output
class Mups16MCAsmInfo : public MCAsmInfo
{
public:
  explicit Mups16MCAsmInfo(const Triple &TheTriple);

    bool shouldOmitSectionDirective(StringRef SectionName) const override { return true; }
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_SPARCMCASMINFO_H
