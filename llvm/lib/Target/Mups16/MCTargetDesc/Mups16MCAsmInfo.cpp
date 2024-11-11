#include "Mups16MCAsmInfo.h"

using namespace llvm;

Mups16MCAsmInfo::Mups16MCAsmInfo(const Triple& TheTriple)
{
    CodePointerSize = 2;
    CalleeSaveStackSlotSize = 2;
    CommentString = ";";
    SupportsDebugInformation = false;

    // AsciiDirective = "#d\t";
    // AscizDirective = nullptr;
    // GlobalDirective = "; global symbol\t";

    // Data8bitsDirective = "#d8\t";
    // Data16bitsDirective = "#d16\t";
    // Data32bitsDirective = "#d32\t";

    // HasDotTypeDotSizeDirective = false;
    // HasSingleParameterDotFile = false;
    AlignmentIsInBytes = false;
    IsLittleEndian = false;
    MaxInstLength = 2;
    // DollarIsPC = true;

    HasIdentDirective = false;
    PrivateGlobalPrefix = ".L";
    PrivateLabelPrefix = ".L";
}
