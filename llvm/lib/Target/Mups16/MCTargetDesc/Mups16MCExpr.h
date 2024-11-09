#ifndef LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16MCEXPR_H
#define LLVM_LIB_TARGET_MUPS16_MCTARGETDESC_MUPS16MCEXPR_H

#include "Mups16FixupKinds.h"
#include "llvm/MC/MCExpr.h"

namespace llvm {

class StringRef;
class Mups16MCExpr : public MCTargetExpr {
public:
  enum VariantKind {
    VK_Mups_None,
    VK_Mups_LO,
    VK_Mups_HI,
  };

private:
  const VariantKind Kind;
  const MCExpr *Expr;

  explicit Mups16MCExpr(VariantKind Kind, const MCExpr *Expr)
  : Kind(Kind), Expr(Expr)
  {}

public:
  static const Mups16MCExpr *create(VariantKind Kind, const MCExpr *Expr,
                                    MCContext &Ctx);

  VariantKind getKind() const { return Kind; }

  const MCExpr *getSubExpr() const { return Expr; }

  Mups16::Fixups getFixupKind() const { return getFixupKind(Kind); }

  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;

  bool evaluateAsRelocatableImpl(MCValue &Res,
                                 const MCAsmLayout *Layout,
                                 const MCFixup *Fixup) const override;
  void visitUsedExpr(MCStreamer &Streamer) const override;
  MCFragment *findAssociatedFragment() const override
  {
    return getSubExpr()->findAssociatedFragment();
  }

  void fixELFSymbolsInTLSFixups(MCAssembler &Asm) const override;

  static bool classof(const MCExpr *E)
  {
    return E->getKind() == MCExpr::Target;
  }

  static bool classof(const Mups16MCExpr *) { return true; }

  static VariantKind parseVariantKind(StringRef name);
  static bool printVariantKind(raw_ostream &OS, VariantKind Kind);
  static Mups16::Fixups getFixupKind(VariantKind Kind);
};

} // end namespace llvm

#endif
