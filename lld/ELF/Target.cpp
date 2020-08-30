//===- Target.cpp ---------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Machine-specific things, such as applying relocations, creation of
// GOT or PLT entries, etc., are handled in this file.
//
// Refer the ELF spec for the single letter variables, S, A or P, used
// in this file.
//
// Some functions defined in this file has "relaxTls" as part of their names.
// They do peephole optimization for TLS variables by rewriting instructions.
// They are not part of the ABI but optional optimization, so you can skip
// them if you are not interested in how TLS variables are optimized.
// See the following paper for the details.
//
//   Ulrich Drepper, ELF Handling For Thread-Local Storage
//   http://www.akkadia.org/drepper/tls.pdf
//
//===----------------------------------------------------------------------===//

#include "Target.h"
#include "InputFiles.h"
#include "OutputSections.h"
#include "SymbolTable.h"
#include "Symbols.h"
#include "SyntheticSections.h"
#include "lld/Common/ErrorHandler.h"
#include "llvm/Object/ELF.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::ELF;
using namespace lld;
using namespace lld::elf;

const TargetInfo *elf::target;

std::string lld::toString(RelType type) {
  StringRef s = getELFRelocationTypeName(elf::config->emachine, type);
  if (s == "Unknown")
    return ("Unknown (" + Twine(type) + ")").str();
  return std::string(s);
}

TargetInfo *elf::getTarget() {
  switch (config->emachine) {
  case EM_386:
  case EM_IAMCU:
    return getX86TargetInfo();
  case EM_AARCH64:
    return getAArch64TargetInfo();
  case EM_AMDGPU:
    return getAMDGPUTargetInfo();
  case EM_ARM:
    return getARMTargetInfo();
  case EM_AVR:
    return getAVRTargetInfo();
  case EM_HEXAGON:
    return getHexagonTargetInfo();
  case EM_MIPS:
    switch (config->ekind) {
    case ELF32LEKind:
      return getMipsTargetInfo<ELF32LE>();
    case ELF32BEKind:
      return getMipsTargetInfo<ELF32BE>();
    case ELF64LEKind:
      return getMipsTargetInfo<ELF64LE>();
    case ELF64BEKind:
      return getMipsTargetInfo<ELF64BE>();
    default:
      llvm_unreachable("unsupported MIPS target");
    }
  case EM_MUPS16:
    return getMups16TargetInfo();
  case EM_MSP430:
    return getMSP430TargetInfo();
  case EM_PPC:
    return getPPCTargetInfo();
  case EM_PPC64:
    return getPPC64TargetInfo();
  case EM_RISCV:
    return getRISCVTargetInfo();
  case EM_SPARCV9:
    return getSPARCV9TargetInfo();
  case EM_X86_64:
    return getX86_64TargetInfo();
  }
  llvm_unreachable("unknown target machine");
}

template <class ELFT> static ErrorPlace getErrPlace(const uint8_t *loc) {
  assert(loc != nullptr);
  for (InputSectionBase *d : inputSections) {
    auto *isec = cast<InputSection>(d);
    if (!isec->getParent() || (isec->type & SHT_NOBITS))
      continue;

    const uint8_t *isecLoc =
        Out::bufferStart
            ? (Out::bufferStart + isec->getParent()->offset + isec->outSecOff)
            : isec->data().data();
    if (isecLoc == nullptr) {
      assert(isa<SyntheticSection>(isec) && "No data but not synthetic?");
      continue;
    }
    if (isecLoc <= loc && loc < isecLoc + isec->getSize())
      return {isec, isec->template getLocation<ELFT>(loc - isecLoc) + ": "};
  }
  return {};
}

ErrorPlace elf::getErrorPlace(const uint8_t *loc) {
  switch (config->ekind) {
  case ELF32LEKind:
    return getErrPlace<ELF32LE>(loc);
  case ELF32BEKind:
    return getErrPlace<ELF32BE>(loc);
  case ELF64LEKind:
    return getErrPlace<ELF64LE>(loc);
  case ELF64BEKind:
    return getErrPlace<ELF64BE>(loc);
  default:
    llvm_unreachable("unknown ELF type");
  }
}

TargetInfo::~TargetInfo() {}

int64_t TargetInfo::getImplicitAddend(const uint8_t *buf, RelType type) const {
  return 0;
}

bool TargetInfo::usesOnlyLowPageBits(RelType type) const { return false; }

bool TargetInfo::needsThunk(RelExpr expr, RelType type, const InputFile *file,
                            uint64_t branchAddr, const Symbol &s,
                            int64_t a) const {
  return false;
}

bool TargetInfo::adjustPrologueForCrossSplitStack(uint8_t *loc, uint8_t *end,
                                                  uint8_t stOther) const {
  llvm_unreachable("Target doesn't support split stacks.");
}

bool TargetInfo::inBranchRange(RelType type, uint64_t src, uint64_t dst) const {
  return true;
}

RelExpr TargetInfo::adjustRelaxExpr(RelType type, const uint8_t *data,
                                    RelExpr expr) const {
  return expr;
}

void TargetInfo::relaxGot(uint8_t *loc, const Relocation &rel,
                          uint64_t val) const {
  llvm_unreachable("Should not have claimed to be relaxable");
}

void TargetInfo::relaxTlsGdToLe(uint8_t *loc, const Relocation &rel,
                                uint64_t val) const {
  llvm_unreachable("Should not have claimed to be relaxable");
}

void TargetInfo::relaxTlsGdToIe(uint8_t *loc, const Relocation &rel,
                                uint64_t val) const {
  llvm_unreachable("Should not have claimed to be relaxable");
}

void TargetInfo::relaxTlsIeToLe(uint8_t *loc, const Relocation &rel,
                                uint64_t val) const {
  llvm_unreachable("Should not have claimed to be relaxable");
}

void TargetInfo::relaxTlsLdToLe(uint8_t *loc, const Relocation &rel,
                                uint64_t val) const {
  llvm_unreachable("Should not have claimed to be relaxable");
}

uint64_t TargetInfo::getImageBase() const {
  // Use -image-base if set. Fall back to the target default if not.
  if (config->imageBase)
    return *config->imageBase;
  return config->isPic ? 0 : defaultImageBase;
}
