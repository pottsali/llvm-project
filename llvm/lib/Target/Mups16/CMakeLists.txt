set(LLVM_TARGET_DEFINITIONS Mups16.td)

tablegen(LLVM Mups16GenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM Mups16GenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM Mups16GenCallingConv.inc -gen-callingconv)
tablegen(LLVM Mups16GenDAGISel.inc -gen-dag-isel)
#tablegen(LLVM Mups16GenDisassemblerTables.inc -gen-disassembler)
#tablegen(LLVM Mups16GenFastISel.inc -gen-fast-isel)
#tablegen(LLVM Mups16GenGlobalISel.inc -gen-global-isel)
tablegen(LLVM Mups16GenInstrInfo.inc -gen-instr-info)
#tablegen(LLVM Mups16GenMCCodeEmitter.inc -gen-emitter)
#tablegen(LLVM Mups16GenMCPseudoLowering.inc -gen-pseudo-lowering)
#tablegen(LLVM Mups16GenRegisterBank.inc -gen-register-bank)
tablegen(LLVM Mups16GenRegisterInfo.inc -gen-register-info)
#tablegen(LLVM Mups16GenSubtargetInfo.inc -gen-subtarget)
#tablegen(LLVM Mups16GenExegesis.inc -gen-exegesis)

add_public_tablegen_target(Mups16CommonTableGen)

add_llvm_target(Mups16CodeGen
    #Mups16AnalyzeImmediate.cpp
    #Mups16AsmPrinter.cpp
    #Mups16CallLowering.cpp
    #Mups16CCState.cpp
    #Mups16ConstantIslandPass.cpp
    #Mups16DelaySlotFiller.cpp
    #Mups16ExpandPseudo.cpp
    #Mups16FastISel.cpp
    #Mups16InstrInfo.cpp
    #Mups16InstructionSelector.cpp
    #Mups16ISelDAGToDAG.cpp
    #Mups16ISelLowering.cpp
    Mups16FrameLowering.cpp
    #Mups16LegalizerInfo.cpp
    #Mups16BranchExpansion.cpp
    #Mups16MCInstLower.cpp
    #Mups16MachineFunction.cpp
    #Mups16ModuleISelDAGToDAG.cpp
    #Mups16OptimizePICCall.cpp
    #Mups16Os16.cpp
    #Mups16PreLegalizerCombiner.cpp
    #Mups16RegisterBankInfo.cpp
    Mups16RegisterInfo.cpp
    #Mups16SEFrameLowering.cpp
    #Mups16SEInstrInfo.cpp
    #Mups16SEISelDAGToDAG.cpp
    #Mups16SEISelLowering.cpp
    #Mups16SERegisterInfo.cpp
    #Mups16Subtarget.cpp
    Mups16TargetMachine.cpp
    #Mups16TargetObjectFile.cpp
    #MicroMups16SizeReduction.cpp
    )

add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
add_subdirectory(AsmParser)
