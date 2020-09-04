//===-- Mups16ISelLowering.cpp - Mups16 DAG Lowering Implementation  ------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the Mups16TargetLowering class.
//
//===----------------------------------------------------------------------===//

#include "Mups16ISelLowering.h"
#include "Mups16.h"
//#include "Mups16MachineFunctionInfo.h"
#include "Mups16Subtarget.h"
#include "Mups16TargetMachine.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalAlias.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "mups16-lower"

Mups16TargetLowering::Mups16TargetLowering(const TargetMachine &TM,
                                           const Mups16Subtarget &STI)
    : TargetLowering(TM) {

  // Set up the register classes.
  addRegisterClass(MVT::i16,  &MUPS::IntRegsRegClass);

  // Compute derived properties from the register classes
  computeRegisterProperties(STI.getRegisterInfo());

  // Provide all sorts of operation actions
  setStackPointerRegisterToSaveRestore(MUPS::SP);
  setBooleanContents(ZeroOrOneBooleanContent);
  setBooleanVectorContents(ZeroOrOneBooleanContent); // FIXME: Is this correct?

  // We have post-incremented loads / stores.
  setIndexedLoadAction(ISD::POST_INC, MVT::i8,  Expand);
  setIndexedLoadAction(ISD::POST_INC, MVT::i16, Expand);

  for (MVT VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD,  VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1,  Promote);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i8,  Expand);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i16, Expand);
  }

  // We don't have any truncstores
  setTruncStoreAction(MVT::i16, MVT::i8, Expand);

  setOperationAction(ISD::SRA,              MVT::i8,    Promote);
  setOperationAction(ISD::SHL,              MVT::i8,    Promote);
  setOperationAction(ISD::SRL,              MVT::i8,    Promote);
  setOperationAction(ISD::ROTL,             MVT::i8,    Expand);
  setOperationAction(ISD::ROTR,             MVT::i8,    Expand);
  setOperationAction(ISD::ROTL,             MVT::i16,   Expand);
  setOperationAction(ISD::ROTR,             MVT::i16,   Expand);
  setOperationAction(ISD::GlobalAddress,    MVT::i16,   Custom);
  setOperationAction(ISD::ExternalSymbol,   MVT::i16,   Custom);
  setOperationAction(ISD::BlockAddress,     MVT::i16,   Custom);
  setOperationAction(ISD::BR_JT,            MVT::Other, Expand);
  setOperationAction(ISD::BR_CC,            MVT::i8,    Promote);
  //setOperationAction(ISD::BR_CC,            MVT::i16,   Custom);
  setOperationAction(ISD::BRCOND,           MVT::Other, Expand);
  setOperationAction(ISD::SETCC,            MVT::i8,    Promote);
  //setOperationAction(ISD::SETCC,            MVT::i16,   Custom);
  setOperationAction(ISD::SELECT,           MVT::i8,    Expand);
  setOperationAction(ISD::SELECT,           MVT::i16,   Expand);
  setOperationAction(ISD::SELECT_CC,        MVT::i8,    Expand);
  setOperationAction(ISD::SELECT_CC,        MVT::i16,   Expand);
  setOperationAction(ISD::SIGN_EXTEND,      MVT::i16,   Custom);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i8, Expand);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i16, Expand);
  setOperationAction(ISD::STACKSAVE,        MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE,     MVT::Other, Expand);

  setOperationAction(ISD::CTTZ,             MVT::i8,    Expand);
  setOperationAction(ISD::CTTZ,             MVT::i16,   Expand);
  setOperationAction(ISD::CTLZ,             MVT::i8,    Expand);
  setOperationAction(ISD::CTLZ,             MVT::i16,   Expand);
  setOperationAction(ISD::CTPOP,            MVT::i8,    Expand);
  setOperationAction(ISD::CTPOP,            MVT::i16,   Expand);

  setOperationAction(ISD::SHL_PARTS,        MVT::i8,    Expand);
  setOperationAction(ISD::SHL_PARTS,        MVT::i16,   Expand);
  setOperationAction(ISD::SRL_PARTS,        MVT::i8,    Expand);
  setOperationAction(ISD::SRL_PARTS,        MVT::i16,   Expand);
  setOperationAction(ISD::SRA_PARTS,        MVT::i8,    Expand);
  setOperationAction(ISD::SRA_PARTS,        MVT::i16,   Expand);

  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1,   Expand);

  setOperationAction(ISD::MUL,              MVT::i8,    Promote);
  setOperationAction(ISD::MULHS,            MVT::i8,    Promote);
  setOperationAction(ISD::MULHU,            MVT::i8,    Promote);
  setOperationAction(ISD::SMUL_LOHI,        MVT::i8,    Promote);
  setOperationAction(ISD::UMUL_LOHI,        MVT::i8,    Promote);
  setOperationAction(ISD::MUL,              MVT::i16,   LibCall);
  setOperationAction(ISD::MULHS,            MVT::i16,   Expand);
  setOperationAction(ISD::MULHU,            MVT::i16,   Expand);
  setOperationAction(ISD::SMUL_LOHI,        MVT::i16,   Expand);
  setOperationAction(ISD::UMUL_LOHI,        MVT::i16,   Expand);

  setOperationAction(ISD::UDIV,             MVT::i8,    Promote);
  setOperationAction(ISD::UDIVREM,          MVT::i8,    Promote);
  setOperationAction(ISD::UREM,             MVT::i8,    Promote);
  setOperationAction(ISD::SDIV,             MVT::i8,    Promote);
  setOperationAction(ISD::SDIVREM,          MVT::i8,    Promote);
  setOperationAction(ISD::SREM,             MVT::i8,    Promote);
  setOperationAction(ISD::UDIV,             MVT::i16,   LibCall);
  setOperationAction(ISD::UDIVREM,          MVT::i16,   Expand);
  setOperationAction(ISD::UREM,             MVT::i16,   LibCall);
  setOperationAction(ISD::SDIV,             MVT::i16,   LibCall);
  setOperationAction(ISD::SDIVREM,          MVT::i16,   Expand);
  setOperationAction(ISD::SREM,             MVT::i16,   LibCall);

  // varargs support
  setOperationAction(ISD::VASTART,          MVT::Other, Custom);
  setOperationAction(ISD::VAARG,            MVT::Other, Expand);
  setOperationAction(ISD::VAEND,            MVT::Other, Expand);
  setOperationAction(ISD::VACOPY,           MVT::Other, Expand);
  setOperationAction(ISD::JumpTable,        MVT::i16,   Custom);

  /*
  // EABI Libcalls - EABI Section 6.2
  const struct {
    const RTLIB::Libcall Op;
    const char * const Name;
    const ISD::CondCode Cond;
  } LibraryCalls[] = {
    // Floating point conversions - EABI Table 6
    { RTLIB::FPROUND_F64_F32,   "__mspabi_cvtdf",   ISD::SETCC_INVALID },
    { RTLIB::FPEXT_F32_F64,     "__mspabi_cvtfd",   ISD::SETCC_INVALID },
    // The following is NOT implemented in libgcc
    //{ RTLIB::FPTOSINT_F64_I16,  "__mspabi_fixdi", ISD::SETCC_INVALID },
    { RTLIB::FPTOSINT_F64_I32,  "__mspabi_fixdli",  ISD::SETCC_INVALID },
    { RTLIB::FPTOSINT_F64_I64,  "__mspabi_fixdlli", ISD::SETCC_INVALID },
    // The following is NOT implemented in libgcc
    //{ RTLIB::FPTOUINT_F64_I16,  "__mspabi_fixdu", ISD::SETCC_INVALID },
    { RTLIB::FPTOUINT_F64_I32,  "__mspabi_fixdul",  ISD::SETCC_INVALID },
    { RTLIB::FPTOUINT_F64_I64,  "__mspabi_fixdull", ISD::SETCC_INVALID },
    // The following is NOT implemented in libgcc
    //{ RTLIB::FPTOSINT_F32_I16,  "__mspabi_fixfi", ISD::SETCC_INVALID },
    { RTLIB::FPTOSINT_F32_I32,  "__mspabi_fixfli",  ISD::SETCC_INVALID },
    { RTLIB::FPTOSINT_F32_I64,  "__mspabi_fixflli", ISD::SETCC_INVALID },
    // The following is NOT implemented in libgcc
    //{ RTLIB::FPTOUINT_F32_I16,  "__mspabi_fixfu", ISD::SETCC_INVALID },
    { RTLIB::FPTOUINT_F32_I32,  "__mspabi_fixful",  ISD::SETCC_INVALID },
    { RTLIB::FPTOUINT_F32_I64,  "__mspabi_fixfull", ISD::SETCC_INVALID },
    // TODO The following IS implemented in libgcc
    //{ RTLIB::SINTTOFP_I16_F64,  "__mspabi_fltid", ISD::SETCC_INVALID },
    { RTLIB::SINTTOFP_I32_F64,  "__mspabi_fltlid",  ISD::SETCC_INVALID },
    // TODO The following IS implemented in libgcc but is not in the EABI
    { RTLIB::SINTTOFP_I64_F64,  "__mspabi_fltllid", ISD::SETCC_INVALID },
    // TODO The following IS implemented in libgcc
    //{ RTLIB::UINTTOFP_I16_F64,  "__mspabi_fltud", ISD::SETCC_INVALID },
    { RTLIB::UINTTOFP_I32_F64,  "__mspabi_fltuld",  ISD::SETCC_INVALID },
    // The following IS implemented in libgcc but is not in the EABI
    { RTLIB::UINTTOFP_I64_F64,  "__mspabi_fltulld", ISD::SETCC_INVALID },
    // TODO The following IS implemented in libgcc
    //{ RTLIB::SINTTOFP_I16_F32,  "__mspabi_fltif", ISD::SETCC_INVALID },
    { RTLIB::SINTTOFP_I32_F32,  "__mspabi_fltlif",  ISD::SETCC_INVALID },
    // TODO The following IS implemented in libgcc but is not in the EABI
    { RTLIB::SINTTOFP_I64_F32,  "__mspabi_fltllif", ISD::SETCC_INVALID },
    // TODO The following IS implemented in libgcc
    //{ RTLIB::UINTTOFP_I16_F32,  "__mspabi_fltuf", ISD::SETCC_INVALID },
    { RTLIB::UINTTOFP_I32_F32,  "__mspabi_fltulf",  ISD::SETCC_INVALID },
    // The following IS implemented in libgcc but is not in the EABI
    { RTLIB::UINTTOFP_I64_F32,  "__mspabi_fltullf", ISD::SETCC_INVALID },

    // Floating point comparisons - EABI Table 7
    { RTLIB::OEQ_F64, "__mspabi_cmpd", ISD::SETEQ },
    { RTLIB::UNE_F64, "__mspabi_cmpd", ISD::SETNE },
    { RTLIB::OGE_F64, "__mspabi_cmpd", ISD::SETGE },
    { RTLIB::OLT_F64, "__mspabi_cmpd", ISD::SETLT },
    { RTLIB::OLE_F64, "__mspabi_cmpd", ISD::SETLE },
    { RTLIB::OGT_F64, "__mspabi_cmpd", ISD::SETGT },
    { RTLIB::OEQ_F32, "__mspabi_cmpf", ISD::SETEQ },
    { RTLIB::UNE_F32, "__mspabi_cmpf", ISD::SETNE },
    { RTLIB::OGE_F32, "__mspabi_cmpf", ISD::SETGE },
    { RTLIB::OLT_F32, "__mspabi_cmpf", ISD::SETLT },
    { RTLIB::OLE_F32, "__mspabi_cmpf", ISD::SETLE },
    { RTLIB::OGT_F32, "__mspabi_cmpf", ISD::SETGT },

    // Floating point arithmetic - EABI Table 8
    { RTLIB::ADD_F64,  "__mspabi_addd", ISD::SETCC_INVALID },
    { RTLIB::ADD_F32,  "__mspabi_addf", ISD::SETCC_INVALID },
    { RTLIB::DIV_F64,  "__mspabi_divd", ISD::SETCC_INVALID },
    { RTLIB::DIV_F32,  "__mspabi_divf", ISD::SETCC_INVALID },
    { RTLIB::MUL_F64,  "__mspabi_mpyd", ISD::SETCC_INVALID },
    { RTLIB::MUL_F32,  "__mspabi_mpyf", ISD::SETCC_INVALID },
    { RTLIB::SUB_F64,  "__mspabi_subd", ISD::SETCC_INVALID },
    { RTLIB::SUB_F32,  "__mspabi_subf", ISD::SETCC_INVALID },
    // The following are NOT implemented in libgcc
    // { RTLIB::NEG_F64,  "__mspabi_negd", ISD::SETCC_INVALID },
    // { RTLIB::NEG_F32,  "__mspabi_negf", ISD::SETCC_INVALID },

    // Universal Integer Operations - EABI Table 9
    { RTLIB::SDIV_I16,   "__mspabi_divi", ISD::SETCC_INVALID },
    { RTLIB::SDIV_I32,   "__mspabi_divli", ISD::SETCC_INVALID },
    { RTLIB::SDIV_I64,   "__mspabi_divlli", ISD::SETCC_INVALID },
    { RTLIB::UDIV_I16,   "__mspabi_divu", ISD::SETCC_INVALID },
    { RTLIB::UDIV_I32,   "__mspabi_divul", ISD::SETCC_INVALID },
    { RTLIB::UDIV_I64,   "__mspabi_divull", ISD::SETCC_INVALID },
    { RTLIB::SREM_I16,   "__mspabi_remi", ISD::SETCC_INVALID },
    { RTLIB::SREM_I32,   "__mspabi_remli", ISD::SETCC_INVALID },
    { RTLIB::SREM_I64,   "__mspabi_remlli", ISD::SETCC_INVALID },
    { RTLIB::UREM_I16,   "__mspabi_remu", ISD::SETCC_INVALID },
    { RTLIB::UREM_I32,   "__mspabi_remul", ISD::SETCC_INVALID },
    { RTLIB::UREM_I64,   "__mspabi_remull", ISD::SETCC_INVALID },

    // Bitwise Operations - EABI Table 10
    // TODO: __mspabi_[srli/srai/slli] ARE implemented in libgcc
    { RTLIB::SRL_I32,    "__mspabi_srll", ISD::SETCC_INVALID },
    { RTLIB::SRA_I32,    "__mspabi_sral", ISD::SETCC_INVALID },
    { RTLIB::SHL_I32,    "__mspabi_slll", ISD::SETCC_INVALID },
    // __mspabi_[srlll/srall/sllll/rlli/rlll] are NOT implemented in libgcc

  };

  for (const auto &LC : LibraryCalls) {
    setLibcallName(LC.Op, LC.Name);
    if (LC.Cond != ISD::SETCC_INVALID)
      setCmpLibcallCC(LC.Op, LC.Cond);
  }

    const struct {
      const RTLIB::Libcall Op;
      const char * const Name;
    } LibraryCalls[] = {
      // Integer Multiply - EABI Table 9
      { RTLIB::MUL_I16,   "__mspabi_mpyi" },
      { RTLIB::MUL_I32,   "__mspabi_mpyl" },
      { RTLIB::MUL_I64,   "__mspabi_mpyll" },
      // The __mspabi_mpysl* functions are NOT implemented in libgcc
      // The __mspabi_mpyul* functions are NOT implemented in libgcc
    };
    for (const auto &LC : LibraryCalls) {
      setLibcallName(LC.Op, LC.Name);
    }
    setLibcallCallingConv(RTLIB::MUL_I64, CallingConv::Mups16_BUILTIN);
  */

  setMinFunctionAlignment(Align(2));
  setPrefFunctionAlignment(Align(2));
}

bool Mups16TargetLowering::isLegalICmpImmediate(int64_t Immed) const {
    return false;
}

//===----------------------------------------------------------------------===//
//                       Mups16 Inline Assembly Support
//===----------------------------------------------------------------------===//

/// getConstraintType - Given a constraint letter, return the type of
/// constraint it is for this target.
TargetLowering::ConstraintType
Mups16TargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    case 'r':
      return C_RegisterClass;
    default:
      break;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}

std::pair<unsigned, const TargetRegisterClass *>
Mups16TargetLowering::getRegForInlineAsmConstraint(
    const TargetRegisterInfo *TRI, StringRef Constraint, MVT VT) const {
  if (Constraint.size() == 1) {
    // GCC Constraint Letters
    switch (Constraint[0]) {
    default: break;
    case 'r':   // GENERAL_REGS
      return std::make_pair(0U, &MUPS::IntRegsRegClass);
    }
  }

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

#include "Mups16GenCallingConv.inc"

bool Mups16TargetLowering::isZExtFree(Type *Ty1, Type *Ty2) const {
    return false;
}

bool Mups16TargetLowering::isZExtFree(EVT VT1, EVT VT2) const {
    return false;
}

bool Mups16TargetLowering::isZExtFree(SDValue Val, EVT VT2) const {
    return false;
}

//===----------------------------------------------------------------------===//
//  Other Lowering Code
//===----------------------------------------------------------------------===//

