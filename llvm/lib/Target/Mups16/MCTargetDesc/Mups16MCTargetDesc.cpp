#include "Mups16MCTargetDesc.h"
#include "Mups16InstPrinter.h"
#include "Mups16MCAsmInfo.h"
#include "TargetInfo/Mups16TargetInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
//#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "Mups16GenInstrInfo.inc"

//#define GET_SUBTARGETINFO_MC_DESC
//#include "Mups16GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "Mups16GenRegisterInfo.inc"

static MCRegisterInfo *createMups16MCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitMups16MCRegisterInfo(X, MUPS::RA);
  return X;
}

static MCInstPrinter *createMups16MCInstPrinter(const Triple &T,
                                                unsigned SyntaxVariant,
                                                const MCAsmInfo &MAI,
                                                const MCInstrInfo &MII,
                                                const MCRegisterInfo &MRI) {
  if (SyntaxVariant == 0)
    return new Mups16InstPrinter(MAI, MII, MRI);
  return nullptr;
}

extern "C" void LLVMInitializeMups16TargetMC()
{
  Target &T = getTheMups16Target();
  RegisterMCAsmInfo<Mups16MCAsmInfo> X(T);
  TargetRegistry::RegisterMCRegInfo(T, createMups16MCRegisterInfo);
  TargetRegistry::RegisterMCInstPrinter(T, createMups16MCInstPrinter);
}
