//===- Mups16AsmParser.cpp - Parse Mups16 assembly to MCInst instructions -===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Mups16.h"
#include "Mups16RegisterInfo.h"
#include "MCTargetDesc/Mups16MCTargetDesc.h"
#include "TargetInfo/Mups16TargetInfo.h"

#include "llvm/ADT/APInt.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

#define DEBUG_TYPE "msp430-asm-parser"

using namespace llvm;

namespace {

/// Parses Mups16 assembly from a stream.
class Mups16AsmParser : public MCTargetAsmParser {
  const MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;
  OperandMatchResultTy tryParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                        SMLoc &EndLoc) override;

  bool mnemonicIsValid(StringRef Mnemonic, unsigned VariantID);

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  bool parseMemOperand(OperandVector &Operands, const MCExpr *Val, SMLoc StartLoc);
  bool parseOperand(OperandVector &, StringRef Mnemonic);

  bool ParseLiteralValues(unsigned Size, SMLoc L);

  /// @name Auto-generated Matcher Functions
  /// {

#define GET_ASSEMBLER_HEADER
#include "Mups16GenAsmMatcher.inc"

  /// }

public:
  Mups16AsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
                  const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), STI(STI), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
  }
};

/// A parsed Mups16 assembly operand.
class Mups16Operand : public MCParsedAsmOperand {
  typedef MCParsedAsmOperand Base;

  enum KindTy {
    k_Imm,
    k_Reg,
    k_Tok,
    k_Mem,
    //k_IndReg,
    //k_PostIndReg
  } Kind;

  struct Memory {
    unsigned Reg;
    const MCExpr *Offset;
  };
  union {
    const MCExpr *Imm;
    unsigned      Reg;
    StringRef     Tok;
    Memory        Mem;
  };

  SMLoc Start, End;

public:
  Mups16Operand(StringRef Tok, SMLoc const &S)
      : Base(), Kind(k_Tok), Tok(Tok), Start(S), End(S) {}
  Mups16Operand(KindTy Kind, unsigned Reg, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(Kind), Reg(Reg), Start(S), End(E) {}
  Mups16Operand(MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Imm), Imm(Imm), Start(S), End(E) {}
  Mups16Operand(unsigned Reg, MCExpr const *Expr, SMLoc const &S, SMLoc const &E)
      : Base(), Kind(k_Mem), Mem({Reg, Expr}), Start(S), End(E) {}

  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Reg && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(Reg));
  }

  void addExprOperand(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediate when possible
    if (!Expr)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Imm && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    addExprOperand(Inst, Imm);
  }

  void addMemOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Mem && "Unexpected operand kind");
    assert(N == 2 && "Invalid number of operands");

    Inst.addOperand(MCOperand::createReg(Mem.Reg));
    addExprOperand(Inst, Mem.Offset);
  }

  bool isReg()   const override { return Kind == k_Reg; }
  bool isImm()   const override { return Kind == k_Imm; }
  bool isToken() const override { return Kind == k_Tok; }
  bool isMem()   const override { return Kind == k_Mem; }
  /*
  bool isIndReg()         const { return Kind == k_IndReg; }
  bool isPostIndReg()     const { return Kind == k_PostIndReg; }

  bool isCGImm() const {
    if (Kind != k_Imm)
      return false;

    int64_t Val;
    if (!Imm->evaluateAsAbsolute(Val))
      return false;
    
    if (Val == 0 || Val == 1 || Val == 2 || Val == 4 || Val == 8 || Val == -1)
      return true;

    return false;
  }
  */

  StringRef getToken() const {
    assert(Kind == k_Tok && "Invalid access!");
    return Tok;
  }

  unsigned getReg() const override {
    assert(Kind == k_Reg && "Invalid access!");
    return Reg;
  }

  /*
  void setReg(unsigned RegNo) {
    assert(Kind == k_Reg && "Invalid access!");
    Reg = RegNo;
  }
  */

  static std::unique_ptr<Mups16Operand> CreateToken(StringRef Str, SMLoc S) {
    return std::make_unique<Mups16Operand>(Str, S);
  }

  static std::unique_ptr<Mups16Operand> CreateReg(unsigned RegNum, SMLoc S,
                                                  SMLoc E) {
    return std::make_unique<Mups16Operand>(k_Reg, RegNum, S, E);
  }

  static std::unique_ptr<Mups16Operand> CreateImm(const MCExpr *Val, SMLoc S,
                                                  SMLoc E) {
    return std::make_unique<Mups16Operand>(Val, S, E);
  }

  static std::unique_ptr<Mups16Operand> CreateMem(unsigned RegNum,
                                                  const MCExpr *Val,
                                                  SMLoc S, SMLoc E) {
    return std::make_unique<Mups16Operand>(RegNum, Val, S, E);
  }

  SMLoc getStartLoc() const override { return Start; }
  SMLoc getEndLoc() const override { return End; }

  void print(raw_ostream &O) const override {
    /*
    switch (Kind) {
    case k_Tok:
      O << "Token " << Tok;
      break;
    case k_Reg:
      O << "Register " << Reg;
      break;
    case k_Imm:
      O << "Immediate " << *Imm;
      break;
    case k_Mem:
      O << "Memory ";
      O << *Mem.Offset << "(" << Reg << ")";
      break;
    case k_IndReg:
      O << "RegInd " << Reg;
      break;
    case k_PostIndReg:
      O << "PostInc " << Reg;
      break;
    }
    */
  }
};
} // end anonymous namespace

bool Mups16AsmParser::MatchAndEmitInstruction(SMLoc Loc, unsigned &Opcode,
                                              OperandVector &Operands,
                                              MCStreamer &Out,
                                              uint64_t &ErrorInfo,
                                              bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);

  switch (MatchResult) {
  case Match_Success:
    Inst.setLoc(Loc);
    Out.emitInstruction(Inst, STI);
    return false;
  case Match_MnemonicFail:
    return Error(Loc, "invalid instruction mnemonic");
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = Loc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");

      ErrorLoc = ((Mups16Operand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = Loc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }
  default:
    return true;
  }
}

// Auto-generated by TableGen
static unsigned MatchRegisterName(StringRef Name);
static std::string Mups16MnemonicSpellCheck(StringRef S, const FeatureBitset &FBS, unsigned VariantID = 0);

bool Mups16AsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                    SMLoc &EndLoc) {
  switch (tryParseRegister(RegNo, StartLoc, EndLoc)) {
  case MatchOperand_ParseFail:
    return Error(StartLoc, "invalid register name");
  case MatchOperand_Success:
    return false;
  case MatchOperand_NoMatch:
    return true;
  }

  llvm_unreachable("unknown match result type");
}

OperandMatchResultTy Mups16AsmParser::tryParseRegister(unsigned &RegNo,
                                                       SMLoc &StartLoc,
                                                       SMLoc &EndLoc) {
  if (getLexer().getKind() != AsmToken::Dollar)
    return MatchOperand_NoMatch;

  AsmToken const &NameToken = getLexer().peekTok(false);
  if (NameToken.getKind() != AsmToken::Identifier)
    return MatchOperand_ParseFail;

  auto Name = NameToken.getIdentifier().upper();
  RegNo = MatchRegisterName(Name);

  if (RegNo == MUPS::NoRegister)
    return MatchOperand_ParseFail;

  StartLoc = getParser().getTok().getLoc();
  EndLoc = NameToken.getEndLoc();
  getLexer().Lex(); // eat $
  getLexer().Lex(); // eat register name

  return MatchOperand_Success;
}

/*
bool Mups16AsmParser::parseJccInstruction(ParseInstructionInfo &Info,
                                          StringRef Name, SMLoc NameLoc,
                                          OperandVector &Operands) {
  if (!Name.startswith_lower("j"))
    return true;

  auto CC = Name.drop_front().lower();
  unsigned CondCode;
  if (CC == "ne" || CC == "nz")
    CondCode = Mups16CC::COND_NE;
  else if (CC == "eq" || CC == "z")
    CondCode = Mups16CC::COND_E;
  else if (CC == "lo" || CC == "nc")
    CondCode = Mups16CC::COND_LO;
  else if (CC == "hs" || CC == "c")
    CondCode = Mups16CC::COND_HS;
  else if (CC == "n")
    CondCode = Mups16CC::COND_N;
  else if (CC == "ge")
    CondCode = Mups16CC::COND_GE;
  else if (CC == "l")
    CondCode = Mups16CC::COND_L;
  else if (CC == "mp")
    CondCode = Mups16CC::COND_NONE;
  else
    return Error(NameLoc, "unknown instruction");

  if (CondCode == (unsigned)Mups16CC::COND_NONE)
    Operands.push_back(Mups16Operand::CreateToken("jmp", NameLoc));
  else {
    Operands.push_back(Mups16Operand::CreateToken("j", NameLoc));
    const MCExpr *CCode = MCConstantExpr::create(CondCode, getContext());
    Operands.push_back(Mups16Operand::CreateImm(CCode, SMLoc(), SMLoc()));
  }

  // Skip optional '$' sign.
  if (getLexer().getKind() == AsmToken::Dollar)
    getLexer().Lex(); // Eat '$'

  const MCExpr *Val;
  SMLoc ExprLoc = getLexer().getLoc();
  if (getParser().parseExpression(Val))
    return Error(ExprLoc, "expected expression operand");

  int64_t Res;
  if (Val->evaluateAsAbsolute(Res))
    if (Res < -512 || Res > 511)
      return Error(ExprLoc, "invalid jump offset");

  Operands.push_back(Mups16Operand::CreateImm(Val, ExprLoc,
    getLexer().getLoc()));

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    getParser().eatToEndOfStatement();
    return Error(Loc, "unexpected token");
  }

  getParser().Lex(); // Consume the EndOfStatement.
  return false;
}

*/

bool Mups16AsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                       StringRef Name, SMLoc NameLoc,
                                       OperandVector &Operands) {
  MCAsmParser &Parser = getParser();
  LLVM_DEBUG(dbgs() << "ParseInstruction\n");

  // Check if we have valid mnemonic
  if (!mnemonicIsValid(Name, 0)) {
    FeatureBitset FBS = ComputeAvailableFeatures(getSTI().getFeatureBits());
    std::string Suggestion = Mups16MnemonicSpellCheck(Name, FBS);
    return Error(NameLoc, "unknown instruction" + Suggestion);
  }
  // First operand in MCInst is instruction mnemonic.
  Operands.push_back(Mups16Operand::CreateToken(Name, NameLoc));

  // Read the remaining operands.
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    // Read the first operand.
    if (parseOperand(Operands, Name)) {
      SMLoc Loc = getLexer().getLoc();
      return Error(Loc, "unexpected token in argument list");
    }

    while (getLexer().is(AsmToken::Comma)) {
      Parser.Lex(); // Eat the comma.
      // Parse and remember the operand.
      if (parseOperand(Operands, Name)) {
        SMLoc Loc = getLexer().getLoc();
        return Error(Loc, "unexpected token in argument list");
      }
    }
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    return Error(Loc, "unexpected token in argument list");
  }
  Parser.Lex(); // Consume the EndOfStatement.
  return false;
}

/*
bool Mups16AsmParser::ParseDirectiveRefSym(AsmToken DirectiveID) {
    StringRef Name;
    if (getParser().parseIdentifier(Name))
      return TokError("expected identifier in directive");

    MCSymbol *Sym = getContext().getOrCreateSymbol(Name);
    getStreamer().emitSymbolAttribute(Sym, MCSA_Global);
    return false;
}

*/
bool Mups16AsmParser::ParseDirective(AsmToken DirectiveID) {
  StringRef IDVal = DirectiveID.getIdentifier();
  if (IDVal.lower() == ".long") {
    ParseLiteralValues(4, DirectiveID.getLoc());
  } else if (IDVal.lower() == ".word" || IDVal.lower() == ".short") {
    ParseLiteralValues(2, DirectiveID.getLoc());
  } else if (IDVal.lower() == ".byte") {
    ParseLiteralValues(1, DirectiveID.getLoc());
//  } else if (IDVal.lower() == ".refsym") {
//    return ParseDirectiveRefSym(DirectiveID);
  }
  return true;
}

bool Mups16AsmParser::parseMemOperand(OperandVector &Operands, const MCExpr *Val, SMLoc StartLoc) {
    getLexer().Lex(); // Eat '('
    SMLoc RegStartLoc, RegEndLoc;
    unsigned RegNo = MUPS::NoRegister;
    if (ParseRegister(RegNo, RegStartLoc, RegEndLoc))
      return true;
    if (getLexer().getKind() != AsmToken::RParen)
      return true;
    SMLoc EndLoc = getParser().getTok().getEndLoc();
    getLexer().Lex(); // Eat ')'
    Operands.push_back(Mups16Operand::CreateMem(RegNo, Val, StartLoc, EndLoc));
    return false;
}

bool Mups16AsmParser::parseOperand(OperandVector &Operands, StringRef Mnemonic) {
  MCAsmParser &Parser = getParser();
  LLVM_DEBUG(dbgs() << "parseOperand\n");

  switch (getLexer().getKind()) {
  case AsmToken::Dollar: {
    unsigned RegNo;
    SMLoc StartLoc, EndLoc;
    if (!ParseRegister(RegNo, StartLoc, EndLoc)) {
        Operands.push_back(Mups16Operand::CreateReg(RegNo, StartLoc, EndLoc));
        return false;
    }
    break;
  }
  case AsmToken::Integer:
  case AsmToken::Plus:
  case AsmToken::Minus: {
    SMLoc StartLoc = getParser().getTok().getLoc();
    const MCExpr *Val;
    if (!getParser().parseExpression(Val)) {
      SMLoc EndLoc = getParser().getTok().getLoc();
      // Try ($rN)
      if (getLexer().getKind() == AsmToken::LParen) {
          if (parseMemOperand(Operands, Val, StartLoc)) {
              return true;
          }
      } else {
        Operands.push_back(Mups16Operand::CreateImm(Val, StartLoc, EndLoc));
      }
      return false;
    }
    return true;
  }
  case AsmToken::LParen: {
    const auto Val = MCConstantExpr::create(0, getContext());
    return parseMemOperand(Operands, Val, getParser().getTok().getLoc());
  }
  default: {
    LLVM_DEBUG(dbgs() << ".. generic integer expression\n");

    const MCExpr *Expr;
    SMLoc S = Parser.getTok().getLoc(); // Start location of the operand.
    if (getParser().parseExpression(Expr))
      return true;

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

    Operands.push_back(Mups16Operand::CreateImm(Expr, S, E));
    return false;
  }
  } // switch(getLexer().getKind())
  return true;
}

bool Mups16AsmParser::ParseLiteralValues(unsigned Size, SMLoc L) {
  auto parseOne = [&]() -> bool {
    const MCExpr *Value;
    if (getParser().parseExpression(Value))
      return true;
    getParser().getStreamer().emitValue(Value, Size, L);
    return false;
  };
  return (parseMany(parseOne));
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMups16AsmParser() {
  RegisterMCAsmParser<Mups16AsmParser> X(getTheMups16Target());
}

/*
static unsigned convertGR16ToGR8(unsigned Reg) {
  switch (Reg) {
  default:
    llvm_unreachable("Unknown GR16 register");
  case Mups16::PC:  return Mups16::PCB;
  case Mups16::SP:  return Mups16::SPB;
  case Mups16::SR:  return Mups16::SRB;
  case Mups16::CG:  return Mups16::CGB;
  case Mups16::R4:  return Mups16::R4B;
  case Mups16::R5:  return Mups16::R5B;
  case Mups16::R6:  return Mups16::R6B;
  case Mups16::R7:  return Mups16::R7B;
  case Mups16::R8:  return Mups16::R8B;
  case Mups16::R9:  return Mups16::R9B;
  case Mups16::R10: return Mups16::R10B;
  case Mups16::R11: return Mups16::R11B;
  case Mups16::R12: return Mups16::R12B;
  case Mups16::R13: return Mups16::R13B;
  case Mups16::R14: return Mups16::R14B;
  case Mups16::R15: return Mups16::R15B;
  }
}

unsigned Mups16AsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                     unsigned Kind) {
  Mups16Operand &Op = static_cast<Mups16Operand &>(AsmOp);

  if (!Op.isReg())
    return Match_InvalidOperand;

  unsigned Reg = Op.getReg();
  bool isGR16 =
      Mups16MCRegisterClasses[Mups16::GR16RegClassID].contains(Reg);

  if (isGR16 && (Kind == MCK_GR8)) {
    Op.setReg(convertGR16ToGR8(Reg));
    return Match_Success;
  }

  return Match_InvalidOperand;
}
*/

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#define GET_MNEMONIC_SPELL_CHECKER
#include "Mups16GenAsmMatcher.inc"

bool Mups16AsmParser::mnemonicIsValid(StringRef Mnemonic, unsigned VariantID) {
  // Find the appropriate table for this asm variant.
  const MatchEntry *Start, *End;
  switch (VariantID) {
  default: llvm_unreachable("invalid variant!");
  case 0: Start = std::begin(MatchTable0); End = std::end(MatchTable0); break;
  }
  // Search the table.
  auto MnemonicRange = std::equal_range(Start, End, Mnemonic, LessOpcode());
  return MnemonicRange.first != MnemonicRange.second;
}
