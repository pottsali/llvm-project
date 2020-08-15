#include "Mups16MCTargetDesc.h"
//#include "Mups16InstPrinter.h"
#include "TargetInfo/Mups16TargetInfo.h"
//#include "llvm/MC/MCInstrInfo.h"
//#include "llvm/MC/MCRegisterInfo.h"
//#include "llvm/MC/MCSubtargetInfo.h"
//#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "Mups16GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "Mups16GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "Mups16GenRegisterInfo.inc"

extern "C" void LLVMInitializeMups16TargetMC()
{

}
