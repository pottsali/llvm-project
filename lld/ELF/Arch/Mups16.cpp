#include "Symbols.h"
#include "SyntheticSections.h"
#include "Target.h"
#include "lld/Common/ErrorHandler.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/Endian.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::support::endian;
using namespace llvm::ELF;
using namespace lld;
using namespace lld::elf;

namespace {

class Mups16 final : public TargetInfo
{
public:
    Mups16();
    uint32_t calcEFlags() const override;
    RelExpr getRelExpr(RelType type, const Symbol &s,
                       const uint8_t *loc) const override;
    //RelType getDynRel(RelType type) const override;
    void relocate(uint8_t *loc, const Relocation &rel,
                  uint64_t val) const override;
    //void writePltHeader(uint8_t *buf) const override;
    //void writePlt(uint8_t *buf, const Symbol &sym,
                  //uint64_t pltEntryAddr) const override;
};
} // namespace


Mups16::Mups16()
{
    //relativeRel = R_MUPS16_RELATIVE;
    symbolicRel = R_MUPS16_16;

    // 256 byte pages by default
    defaultMaxPageSize = 0x100;
    noneRel = R_MUPS16_NONE;

    defaultImageBase = 0x600;
}

uint32_t Mups16::calcEFlags() const
{
    return 0;
}

RelExpr Mups16::getRelExpr(RelType type, const Symbol &s, const uint8_t *loc) const
{
    switch (type)
    {
    case R_MUPS16_NONE:
        return R_NONE;
    case R_MUPS16_8:
    case R_MUPS16_16:
    case R_MUPS16_LO8:
    case R_MUPS16_HI8:
        return R_ABS;
    // When I add branch/jump relocations these will be R_PC
    //case R_HEX_32_PCREL:
    //  return R_PC;
    default:
        error(getErrorLocation(loc) + "unknown relocation (" + Twine(type) + ") against symbol " + toString(s));
        return R_NONE;
    }
}

void Mups16::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const
{
    switch (rel.type)
    {
    case R_MUPS16_NONE:
        break;
    case R_MUPS16_8:
        break;
    }
}


TargetInfo *elf::getMups16TargetInfo()
{
    static Mups16 target;
    return &target;
}
