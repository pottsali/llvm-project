#include "Mups16MCExpr.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCObjectStreamer.h"
#include "llvm/MC/MCSymbolELF.h"

using namespace llvm;

const Mups16MCExpr*
Mups16MCExpr::create(VariantKind Kind, const MCExpr *Expr,
                     MCContext &Ctx)
{
  return new (Ctx) Mups16MCExpr(Kind, Expr);
}

void Mups16MCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const
{

  bool closeParen = printVariantKind(OS, Kind);

  const MCExpr *Expr = getSubExpr();
  Expr->print(OS, MAI);

  if (closeParen)
    OS << ')';
}

bool Mups16MCExpr::printVariantKind(raw_ostream &OS, VariantKind Kind)
{
  bool closeParen = true;
  switch (Kind) {
    case VK_Mups_None:     closeParen = false; break;
    case VK_Mups_LO:       OS << "%lo(";  break;
    case VK_Mups_HI:       OS << "%hi(";  break;
  }
  return closeParen;
}

Mups16MCExpr::VariantKind Mups16MCExpr::parseVariantKind(StringRef name)
{
  return StringSwitch<Mups16MCExpr::VariantKind>(name)
  .Case("lo",  VK_Mups_LO)
  .Case("hi",  VK_Mups_HI)
  .Default(VK_Mups_None);
}

Mups16::Fixups Mups16MCExpr::getFixupKind(Mups16MCExpr::VariantKind Kind)
{
  switch (Kind) {
    default: llvm_unreachable("Unhandled Mups16MCExpr::VariantKind");
    case VK_Mups_HI:      return Mups16::fixup_mups6_hi8;
    case VK_Mups_LO:      return Mups16::fixup_mups6_lo8;
  }
}
bool Mups16MCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                             const MCAsmLayout *Layout,
                                             const MCFixup *Fixup) const
{
  return getSubExpr()->evaluateAsRelocatable(Res, Layout, Fixup);
}

static void fixELFSymbolsInTLSFixupsImpl(const MCExpr *Expr, MCAssembler &Asm)
{
  // FIXME: what does this do?
}

void Mups16MCExpr::fixELFSymbolsInTLSFixups(MCAssembler &Asm) const
{
  // FIXME: what does this do?
}

void Mups16MCExpr::visitUsedExpr(MCStreamer &Streamer) const
{
  Streamer.visitUsedExpr(*getSubExpr());
}

