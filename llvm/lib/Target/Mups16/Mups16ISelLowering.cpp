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
    : TargetLowering(TM)
{

    // Set up the register classes.
    addRegisterClass(MVT::i16,  &MUPS::IntRegsRegClass);

    // Compute derived properties from the register classes
    computeRegisterProperties(STI.getRegisterInfo());

    // Provide all sorts of operation actions
    setStackPointerRegisterToSaveRestore(MUPS::SP);
    setBooleanContents(ZeroOrOneBooleanContent);
    setBooleanVectorContents(ZeroOrOneBooleanContent); // FIXME: Is this correct?

    // We don't have post-incremented loads / stores.
    setIndexedLoadAction(ISD::POST_INC, MVT::i8,  Expand);
    setIndexedLoadAction(ISD::POST_INC, MVT::i16, Expand);

    for (MVT VT : MVT::integer_valuetypes()) {
        setLoadExtAction(ISD::EXTLOAD,  VT, MVT::i1,  Promote);
        setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1,  Promote);
        setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1,  Promote);
        //setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i8,  Expand);
        //setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i16, Expand);
    }

    // We don't have any truncstores
    //setTruncStoreAction(MVT::i16, MVT::i8, Expand);

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
    setOperationAction(ISD::BR_CC,            MVT::i16,   Expand);
    //setOperationAction(ISD::BR_CC,            MVT::i16,   Custom);
    setOperationAction(ISD::BRCOND,           MVT::Other, Legal);
    setOperationAction(ISD::SETCC,            MVT::i8,    Promote);
    setOperationAction(ISD::SETCC,            MVT::i16,   Legal);
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

    // Valid condcode actions (which comparisons are natively supported by Mups16)
    setCondCodeAction(ISD::SETOLE, MVT::i16, Expand);
    setCondCodeAction(ISD::SETOGE, MVT::i16, Expand);
    setCondCodeAction(ISD::SETOGT, MVT::i16, Expand);
    setCondCodeAction(ISD::SETULE, MVT::i16, Expand);
    setCondCodeAction(ISD::SETUGE, MVT::i16, Expand);
    setCondCodeAction(ISD::SETUGT, MVT::i16, Expand);
    



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


// Custom code for lowering LLVM instructions that Mups16 doesn't natively support, and which can't
// be efficiently implemented by just expanding.
SDValue Mups16TargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const
{
    switch (Op.getOpcode())
    {
    }
    return {};
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
SDValue Mups16TargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                SmallVectorImpl<SDValue> &InVals) const
{
    SelectionDAG &DAG                     = CLI.DAG;
    SDLoc &dl                             = CLI.DL;
    SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
    SmallVectorImpl<SDValue> &OutVals     = CLI.OutVals;
    SmallVectorImpl<ISD::InputArg> &Ins   = CLI.Ins;
    SDValue Chain                         = CLI.Chain;
    SDValue Callee                        = CLI.Callee;
    bool &isTailCall                      = CLI.IsTailCall;
    CallingConv::ID CallConv              = CLI.CallConv;
    bool isVarArg                         = CLI.IsVarArg;

    // MSP430 target does not yet support tail call optimization.
    isTailCall = false;

    switch (CallConv) {
        case CallingConv::Fast:
        case CallingConv::C:
            //return LowerCCCCallTo(Chain, Callee, CallConv, isVarArg, isTailCall,
            //        Outs, OutVals, Ins, dl, DAG, InVals);
        default:
            report_fatal_error("Unsupported calling convention");
    }
}

/*
SDValue Mups16TargetLowering::LowerCCCCallTo(
    SDValue Chain, SDValue Callee, CallingConv::ID CallConv, bool isVarArg,
    bool isTailCall, const SmallVectorImpl<ISD::OutputArg> &Outs,
    const SmallVectorImpl<SDValue> &OutVals,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const
{
    // Analyze operands of the call, assigning locations to each operand.
    SmallVector<CCValAssign, 16> ArgLocs;
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
            *DAG.getContext());
    AnalyzeArguments(CCInfo, ArgLocs, Outs);

    // Get a count of how many bytes are to be pushed on the stack.
    unsigned NumBytes = CCInfo.getNextStackOffset();
    auto PtrVT = getPointerTy(DAG.getDataLayout());

    Chain = DAG.getCALLSEQ_START(Chain, NumBytes, 0, dl);

    SmallVector<std::pair<unsigned, SDValue>, 4> RegsToPass;
    SmallVector<SDValue, 12> MemOpChains;
    SDValue StackPtr;

    // Walk the register/memloc assignments, inserting copies/loads.
    for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
        CCValAssign &VA = ArgLocs[i];

        SDValue Arg = OutVals[i];

        // Promote the value if needed.
        switch (VA.getLocInfo()) {
            default: llvm_unreachable("Unknown loc info!");
            case CCValAssign::Full: break;
            case CCValAssign::SExt:
                                    Arg = DAG.getNode(ISD::SIGN_EXTEND, dl, VA.getLocVT(), Arg);
                                    break;
            case CCValAssign::ZExt:
                                    Arg = DAG.getNode(ISD::ZERO_EXTEND, dl, VA.getLocVT(), Arg);
                                    break;
            case CCValAssign::AExt:
                                    Arg = DAG.getNode(ISD::ANY_EXTEND, dl, VA.getLocVT(), Arg);
                                    break;
        }

        // Arguments that can be passed on register must be kept at RegsToPass
        // vector
        if (VA.isRegLoc()) {
            RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
        } else {
            assert(VA.isMemLoc());

            if (!StackPtr.getNode())
                StackPtr = DAG.getCopyFromReg(Chain, dl, MSP430::SP, PtrVT);

            SDValue PtrOff =
                DAG.getNode(ISD::ADD, dl, PtrVT, StackPtr,
                        DAG.getIntPtrConstant(VA.getLocMemOffset(), dl));

            SDValue MemOp;
            ISD::ArgFlagsTy Flags = Outs[i].Flags;

            if (Flags.isByVal()) {
                SDValue SizeNode = DAG.getConstant(Flags.getByValSize(), dl, MVT::i16);
                MemOp = DAG.getMemcpy(
                        Chain, dl, PtrOff, Arg, SizeNode, Flags.getNonZeroByValAlign(),
                        false, //isVolatile
                        true,  //AlwaysInline
                        false, //isTailCall
                        MachinePointerInfo(), MachinePointerInfo());
            } else {
                MemOp = DAG.getStore(Chain, dl, Arg, PtrOff, MachinePointerInfo());
            }

            MemOpChains.push_back(MemOp);
        }
    }

    // Transform all store nodes into one single node because all store nodes are
    // independent of each other.
    if (!MemOpChains.empty())
        Chain = DAG.getNode(ISD::TokenFactor, dl, MVT::Other, MemOpChains);

    // Build a sequence of copy-to-reg nodes chained together with token chain and
    // flag operands which copy the outgoing args into registers.  The InFlag in
    // necessary since all emitted instructions must be stuck together.
    SDValue InFlag;
    for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
        Chain = DAG.getCopyToReg(Chain, dl, RegsToPass[i].first,
                RegsToPass[i].second, InFlag);
        InFlag = Chain.getValue(1);
    }

    // If the callee is a GlobalAddress node (quite common, every direct call is)
    // turn it into a TargetGlobalAddress node so that legalize doesn't hack it.
    // Likewise ExternalSymbol -> TargetExternalSymbol.
    if (GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee))
        Callee = DAG.getTargetGlobalAddress(G->getGlobal(), dl, MVT::i16);
    else if (ExternalSymbolSDNode *E = dyn_cast<ExternalSymbolSDNode>(Callee))
        Callee = DAG.getTargetExternalSymbol(E->getSymbol(), MVT::i16);

    // Returns a chain & a flag for retval copy to use.
    SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
    SmallVector<SDValue, 8> Ops;
    Ops.push_back(Chain);
    Ops.push_back(Callee);

    // Add argument registers to the end of the list so that they are
    // known live into the call.
    for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i)
        Ops.push_back(DAG.getRegister(RegsToPass[i].first,
                    RegsToPass[i].second.getValueType()));

    if (InFlag.getNode())
        Ops.push_back(InFlag);

    Chain = DAG.getNode(MSP430ISD::CALL, dl, NodeTys, Ops);
    InFlag = Chain.getValue(1);

    // Create the CALLSEQ_END node.
    Chain = DAG.getCALLSEQ_END(Chain, DAG.getConstant(NumBytes, dl, PtrVT, true),
            DAG.getConstant(0, dl, PtrVT, true), InFlag, dl);
    InFlag = Chain.getValue(1);

    // Handle result values, copying them out of physregs into vregs that we
    // return.
    return LowerCallResult(Chain, InFlag, CallConv, isVarArg, Ins, dl,
            DAG, InVals);
}
*/


SDValue Mups16TargetLowering::LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
        const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl, SelectionDAG &DAG,
        SmallVectorImpl<SDValue> &InVals) const
{
    switch (CallConv)
    {
        case CallingConv::C:
        case CallingConv::Fast:
            return LowerCCCArguments(Chain, CallConv, isVarArg, Ins, dl, DAG, InVals);
        default:
            report_fatal_error("Unsupported calling convention");
    }
}

// Generate instructions to load incoming arguments from the stack
SDValue Mups16TargetLowering::LowerCCCArguments(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl, SelectionDAG &DAG, 
    SmallVectorImpl<SDValue> &InVals) const
{
    MachineFunction &MF = DAG.getMachineFunction();
    MachineFrameInfo &MFI = MF.getFrameInfo();
    MachineRegisterInfo &RegInfo = MF.getRegInfo();
    //auto *FuncInfo = MF.getInfo<MachineFunctionInfo>();

    // Assign locations to all of the incoming arguments.
    SmallVector<CCValAssign, 16> ArgLocs;
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs, *DAG.getContext());
    CCInfo.AnalyzeFormalArguments(Ins, CC_Mups16);

    // Create frame index for the start of the first vararg value
    // TODO: varargs
    /*
    if (isVarArg)
    {
        unsigned Offset = CCInfo.getNextStackOffset();
        FuncInfo->setVarArgsFrameIndex(MFI.CreateFixedObject(1, Offset, true));
    }
    */

    for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i)
    {
        CCValAssign &VA = ArgLocs[i];
        if (VA.isRegLoc())
        {
            // Arguments passed in registers
            EVT RegVT = VA.getLocVT();
            switch (RegVT.getSimpleVT().SimpleTy)
            {
                case MVT::i16:
                {
                    Register VReg = RegInfo.createVirtualRegister(&MUPS::IntRegsRegClass);
                    RegInfo.addLiveIn(VA.getLocReg(), VReg);
                    SDValue ArgValue = DAG.getCopyFromReg(Chain, dl, VReg, RegVT);

                    // If this is an 8-bit value, it is really passed promoted to 16
                    // bits. Insert an assert[sz]ext to capture this, then truncate to the
                    // right size.
                    if (VA.getLocInfo() == CCValAssign::SExt)
                        ArgValue = DAG.getNode(ISD::AssertSext, dl, RegVT, ArgValue, DAG.getValueType(VA.getValVT()));
                    else if (VA.getLocInfo() == CCValAssign::ZExt)
                        ArgValue = DAG.getNode(ISD::AssertZext, dl, RegVT, ArgValue, DAG.getValueType(VA.getValVT()));

                    if (VA.getLocInfo() != CCValAssign::Full)
                        ArgValue = DAG.getNode(ISD::TRUNCATE, dl, VA.getValVT(), ArgValue);

                    InVals.push_back(ArgValue);
                    break;
                }
                default:
                {
#ifndef NDEBUG
                    errs() << "LowerFormalArguments Unhandled argument type: "
                        << RegVT.getEVTString() << "\n";
#endif
                    llvm_unreachable(nullptr);
                }
            }
        }
        else
        {
            // Sanity check
            assert(VA.isMemLoc());

            SDValue InVal;
            ISD::ArgFlagsTy Flags = Ins[i].Flags;

            if (Flags.isByVal())
            {
                int FI = MFI.CreateFixedObject(Flags.getByValSize(),
                        VA.getLocMemOffset(), true);
                InVal = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
            }
            else
            {
                // Load the argument to a virtual register
                unsigned ObjSize = VA.getLocVT().getSizeInBits()/8;
                if (ObjSize > 2)
                {
                    errs() << "LowerFormalArguments Unhandled argument type: "
                        << EVT(VA.getLocVT()).getEVTString()
                        << "\n";
                }
                // Create the frame index object for this incoming parameter...
                int FI = MFI.CreateFixedObject(ObjSize, VA.getLocMemOffset(), true);

                // Create the SelectionDAG nodes corresponding to a load
                //from this parameter
                SDValue FIN = DAG.getFrameIndex(FI, MVT::i16);
                InVal = DAG.getLoad(
                        VA.getLocVT(), dl, Chain, FIN,
                        MachinePointerInfo::getFixedStack(DAG.getMachineFunction(), FI));
            }

            InVals.push_back(InVal);
        }
    }

    // TODO: struct returns (isSRet() == true)?

    return Chain;
}

bool Mups16TargetLowering::CanLowerReturn(CallingConv::ID CallConv, MachineFunction &MF,
        bool IsVarArg, const SmallVectorImpl<ISD::OutputArg> &Outs, LLVMContext &Context) const
{
    SmallVector<CCValAssign, 16> RVLocs;
    CCState CCInfo(CallConv, IsVarArg, MF, RVLocs, Context);
    return CCInfo.CheckReturn(Outs, RetCC_Mups16);
}


SDValue Mups16TargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
        bool isVarArg, const SmallVectorImpl<ISD::OutputArg> &Outs,
        const SmallVectorImpl<SDValue> &OutVals, const SDLoc &dl, SelectionDAG &DAG) const
{

    MachineFunction &MF = DAG.getMachineFunction();

    // CCValAssign - represent the assignment of the return value to a location
    SmallVector<CCValAssign, 16> RVLocs;

    // CCState - Info about the registers and stack slot.
    CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs, *DAG.getContext());

    // Analyse return values.
    CCInfo.AnalyzeReturn(Outs, RetCC_Mups16);

    SDValue Glue;
    SmallVector<SDValue, 4> RetOps(1, Chain);

    // Copy the result values into the output registers.
    for (unsigned i = 0; i != RVLocs.size(); ++i)
    {
        CCValAssign &VA = RVLocs[i];
        assert(VA.isRegLoc() && "Can only return in registers!");

        Chain = DAG.getCopyToReg(Chain, dl, VA.getLocReg(), OutVals[i], Glue);

        // Guarantee that all emitted copies are stuck together,
        // avoiding something bad.
        Glue = Chain.getValue(1);
        RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
    }

    /* TODO
    if (MF.getFunction().hasStructRetAttr())
    {
        MSP430MachineFunctionInfo *FuncInfo = MF.getInfo<MSP430MachineFunctionInfo>();
        unsigned Reg = FuncInfo->getSRetReturnReg();

        if (!Reg)
            llvm_unreachable("sret virtual register not created in entry block");

        SDValue Val =
            DAG.getCopyFromReg(Chain, dl, Reg, getPointerTy(DAG.getDataLayout()));
        unsigned R12 = MSP430::R12;

        Chain = DAG.getCopyToReg(Chain, dl, R12, Val, Glue);
        Glue = Chain.getValue(1);
        RetOps.push_back(DAG.getRegister(R12, getPointerTy(DAG.getDataLayout())));
    }
    */

    RetOps[0] = Chain;  // Update chain.

    // Add the flag if we have it.
    if (Glue.getNode())
        RetOps.push_back(Glue);

    return DAG.getNode(Mups16ISD::Ret, dl, MVT::Other, RetOps);
}

const char *Mups16TargetLowering::getTargetNodeName(unsigned Opcode) const
{
    switch ((Mups16ISD::NodeType)Opcode)
    {
        case Mups16ISD::FIRST_NUMBER:
            break;
        case Mups16ISD::JmpLink:
            return "Mups16ISD::JmpLink";
        case Mups16ISD::Ret:
            return "Mups16ISD::Ret";
        case Mups16ISD::LoadImm:
            return "Mups16ISD::LoadImm";
    }

    return nullptr;
}
