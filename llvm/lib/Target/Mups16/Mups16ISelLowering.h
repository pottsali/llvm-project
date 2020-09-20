//===-- Mups16ISelLowering.h - Mups16 DAG Lowering Interface ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that Mups16 uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MUPS16_MUPS16ISELLOWERING_H
#define LLVM_LIB_TARGET_MUPS16_MUPS16ISELLOWERING_H

#include "Mups16.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

  class Mups16Subtarget;
  class Mups16TargetLowering : public TargetLowering {
  public:
    explicit Mups16TargetLowering(const TargetMachine &TM,
                                  const Mups16Subtarget &STI);

    /*
    MVT getScalarShiftAmountTy(const DataLayout &, EVT) const override {
      return MVT::i8;
    }

    MVT::SimpleValueType getCmpLibcallReturnType() const override {
      return MVT::i16;
    }
    */

    TargetLowering::ConstraintType getConstraintType(StringRef Constraint) const override;
    std::pair<unsigned, const TargetRegisterClass *>
    getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                 StringRef Constraint, MVT VT) const override;

    /// isZExtFree - Return true if any actual instruction that defines a value
    /// of type Ty1 implicit zero-extends the value to Ty2 in the result
    /// register. This does not necessarily include registers defined in unknown
    /// ways, such as incoming arguments, or copies from unknown virtual
    /// registers. Also, if isTruncateFree(Ty2, Ty1) is true, this does not
    /// necessarily apply to truncate instructions. e.g. on msp430, all
    /// instructions that define 8-bit values implicit zero-extend the result
    /// out to 16 bits.
    bool isZExtFree(Type *Ty1, Type *Ty2) const override;
    bool isZExtFree(EVT VT1, EVT VT2) const override;
    bool isZExtFree(SDValue Val, EVT VT2) const override;

    bool isLegalICmpImmediate(int64_t) const override;

  private:

    /*
    SDValue LowerCCCCallTo(SDValue Chain, SDValue Callee,
                           CallingConv::ID CallConv, bool isVarArg,
                           bool isTailCall,
                           const SmallVectorImpl<ISD::OutputArg> &Outs,
                           const SmallVectorImpl<SDValue> &OutVals,
                           const SmallVectorImpl<ISD::InputArg> &Ins,
                           const SDLoc &dl, SelectionDAG &DAG,
                           SmallVectorImpl<SDValue> &InVals) const;
                           */

    SDValue LowerCCCArguments(SDValue Chain, CallingConv::ID CallConv,
                              bool isVarArg,
                              const SmallVectorImpl<ISD::InputArg> &Ins,
                              const SDLoc &dl, SelectionDAG &DAG,
                              SmallVectorImpl<SDValue> &InVals) const;

    /*
    SDValue LowerCallResult(SDValue Chain, SDValue InFlag,
                            CallingConv::ID CallConv, bool isVarArg,
                            const SmallVectorImpl<ISD::InputArg> &Ins,
                            const SDLoc &dl, SelectionDAG &DAG,
                            SmallVectorImpl<SDValue> &InVals) const;

    */

    SDValue LowerCall(TargetLowering::CallLoweringInfo &CLI,
                SmallVectorImpl<SDValue> &InVals) const override;
    SDValue LowerFormalArguments(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
                         const SmallVectorImpl<ISD::InputArg> &Ins,
                         const SDLoc &dl, SelectionDAG &DAG,
                         SmallVectorImpl<SDValue> &InVals) const override;

    /*
     * FIXME: these are required
    bool CanLowerReturn(CallingConv::ID CallConv,
                        MachineFunction &MF,
                        bool IsVarArg,
                        const SmallVectorImpl<ISD::OutputArg> &Outs,
                        LLVMContext &Context) const override;

    SDValue LowerReturn(SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
                        const SmallVectorImpl<ISD::OutputArg> &Outs,
                        const SmallVectorImpl<SDValue> &OutVals,
                        const SDLoc &dl, SelectionDAG &DAG) const override;

    */
  };
} // namespace llvm

#endif
