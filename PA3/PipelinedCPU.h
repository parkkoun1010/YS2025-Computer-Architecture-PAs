#ifndef __PIPELINED_CPU_H__
#define __PIPELINED_CPU_H__

#include "DigitalCircuit.h"

#include "Memory.h"
#include "Control.h"
#include "RegisterFile.h"
#include "ALU.h"
#include "ALUControl.h"

#include "Miscellaneous.h"

#ifdef ENABLE_DATA_FORWARDING
class ForwardingUnit : public DigitalCircuit {
  public:
    ForwardingUnit(
      const std::string &name,
      const Wire<5> *iIDEXRs,
      const Wire<5> *iIDEXRt,
      const Wire<1> *iEXMEMRegWrite,
      const Wire<5> *iEXMEMRegDstIdx,
      const Wire<1> *iMEMWBRegWrite,
      const Wire<5> *iMEMWBRegDstIdx,
      Wire<2> *oForwardA,
      Wire<2> *oForwardB
    ) : DigitalCircuit(name) {
      _iIDEXRs = iIDEXRs;
      _iIDEXRt = iIDEXRt;
      _iEXMEMRegWrite = iEXMEMRegWrite;
      _iEXMEMRegDstIdx = iEXMEMRegDstIdx;
      _iMEMWBRegWrite = iMEMWBRegWrite;
      _iMEMWBRegDstIdx = iMEMWBRegDstIdx;
      _oForwardA = oForwardA;
      _oForwardB = oForwardB;
    }
    virtual void advanceCycle() {
      /* FIXME */
      *_oForwardA = 0;
      *_oForwardB = 0;
      unsigned long idex_rs = _iIDEXRs->to_ulong();
      unsigned long idex_rt = _iIDEXRt->to_ulong();
      bool exmem_reg_write = _iEXMEMRegWrite->test(0);
      unsigned long exmem_reg_dst_idx = _iEXMEMRegDstIdx->to_ulong();
      bool memwb_reg_write = _iMEMWBRegWrite->test(0);
      unsigned long memwb_reg_dst_idx = _iMEMWBRegDstIdx->to_ulong();

      // EX hazard for ALU's first operand (rs)
      if (exmem_reg_write && (exmem_reg_dst_idx != 0) && (exmem_reg_dst_idx == idex_rs)){
        *_oForwardA = 0b10;
      }
      // EX hazard for ALU's first operand (rt)
      if (exmem_reg_write && (exmem_reg_dst_idx != 0) && (exmem_reg_dst_idx == idex_rt)){
        *_oForwardB = 0b10;
      }

      if (memwb_reg_write && (memwb_reg_dst_idx != 0) && (memwb_reg_dst_idx == idex_rs)) {
        // Only forward from MEM/WB if not already forwarding from EX/MEM for the same operand
        if (_oForwardA->to_ulong() != 0b10) { // If not already forwarding from EX/MEM for rs
          *_oForwardA = 0b01;
        }
      }

      if (memwb_reg_write && (memwb_reg_dst_idx != 0) && (memwb_reg_dst_idx == idex_rt)) {
        if (_oForwardB->to_ulong() != 0b10) { // If not already forwarding from EX/MEM for rs
          *_oForwardB = 0b01;
        }
      }
        
    }
  private:
    const Wire<5> *_iIDEXRs;
    const Wire<5> *_iIDEXRt;
    const Wire<1> *_iEXMEMRegWrite;
    const Wire<5> *_iEXMEMRegDstIdx;
    const Wire<1> *_iMEMWBRegWrite;
    const Wire<5> *_iMEMWBRegDstIdx;
    Wire<2> *_oForwardA;
    Wire<2> *_oForwardB;
};

#ifdef ENABLE_HAZARD_DETECTION
class HazardDetectionUnit : public DigitalCircuit {
  public:
    HazardDetectionUnit(
      const std::string &name,
      const Wire<5> *iIFIDRs,
      const Wire<5> *iIFIDRt,
      const Wire<1> *iIDEXMemRead,
      const Wire<5> *iIDEXRt,
      Wire<1> *oPCWrite,
      Wire<1> *oIFIDWrite,
      Wire<1> *oIDEXCtrlWrite
    ) : DigitalCircuit(name) {
      _iIFIDRs = iIFIDRs;
      _iIFIDRt = iIFIDRt;
      _iIDEXMemRead = iIDEXMemRead;
      _iIDEXRt = iIDEXRt;
      _oPCWrite = oPCWrite;
      _oIFIDWrite = oIFIDWrite;
      _oIDEXCtrlWrite = oIDEXCtrlWrite;
    }
    virtual void advanceCycle() {
      /* FIXME */
        *_oPCWrite = 1;
        *_oIFIDWrite = 1;
        *_oIDEXCtrlWrite = 1;

        bool idexMemRead = _iIDEXMemRead->test(0);
        unsigned long idexRt = _iIDEXRt->to_ulong();

        unsigned long ifidRs = _iIFIDRs->to_ulong();
        unsigned long ifidRt = _iIFIDRt->to_ulong();

        // ID/EX의 명령어가lw이고,
        // 이 lw의 목적지 레지스터가 $0가 아니면서
        // IF/ID 단계 명령어의 소스레지스터와 같다면 해저드 발생!
        if (idexMemRead && (idexRt != 0) &&
        (idexRt == ifidRs || idexRt == ifidRt)) {
            *_oPCWrite = 0;       // PC 업데이트 금지 (stall)
            *_oIFIDWrite = 0;    // IF/ID 래치 업데이트 금지 (stall)
            *_oIDEXCtrlWrite = 0; // ID/EX 래치에 버블 삽입 (제어 신호 0)
        }
    }
  private:
    const Wire<5> *_iIFIDRs;
    const Wire<5> *_iIFIDRt;
    const Wire<1> *_iIDEXMemRead;
    const Wire<5> *_iIDEXRt;
    Wire<1> *_oPCWrite;
    Wire<1> *_oIFIDWrite;
    Wire<1> *_oIDEXCtrlWrite;
};
#endif // ENABLE_HAZARD_DETECTION
#endif // ENABLE_DATA_FORWARDING

class PipelinedCPU : public DigitalCircuit {

  public:

    PipelinedCPU(
      const std::string &name,
      const std::uint32_t &initialPC,
      const Memory::Endianness &memoryEndianness,
      const char *regFileName,
      const char *instMemFileName,
      const char *dataMemFileName
    ) : DigitalCircuit(name) {
      /* FIXME */
      // Initialize PC and constant wires
      _PC = initialPC;
      _adderPCPlus4Input1 = 4;

      // Initialize current cycle
      _currCycle = 0;

      // IF Stage Components
      // Adder for PC +
      _adderPCPlus4 = new Adder<32>("Adder_PCPlus4", &_PC, &_adderPCPlus4Input1, &_pcPlus4);

      // Instruction Memory
      // Input: _PC (address), Output: _wInstMemOutput (instruction)
      // MemRead is always high, MemWrite is always low for instruction memory
      _instMemory = new Memory("InstMemory", &_PC, nullptr, &_alwaysHi, &_alwaysLo, &_latchIFID_next.instruction, memoryEndianness, instMemFileName);

      // ID Stage Components
      // Control Unit
      // Input: _opcode (from instruction), Outputs: various control signals
      _control = new Control(&_opcode, &_latchIDEX_next.ctrlEX.regDst, &_latchIDEX_next.ctrlEX.aluSrc, &_latchIDEX_next.ctrlWB.memToReg, &_latchIDEX_next.ctrlWB.regWrite, &_latchIDEX_next.ctrlMEM.memRead, &_latchIDEX_next.ctrlMEM.memWrite, &_latchIDEX_next.ctrlMEM.branch, &_latchIDEX_next.ctrlEX.aluOp);

      // Register File
      // Read Ports Inputs: _regFileReadRegister, _regFileReadRegister2 (set in ID stage from instruction)
      // Write Port Inputs: _latchMEMWB.regDstIdx (write address), _muxMemToRegOutput (write data), _latchMEMWB.ctrlWB.regWrite (write enable)
      // Read Ports Outputs: _latchIDEX.regFileReadData1, _latchIDEX.regFileReadData2
      _registerFile = new RegisterFile(&_regFileReadRegister1, &_regFileReadRegister2, &_latchMEMWB.regDstIdx, &_muxMemToRegOutput, &_latchMEMWB.ctrlWB.regWrite, &_latchIDEX_next.regFileReadData1, &_latchIDEX_next.regFileReadData2, regFileName);

      // Sign Extend Unit
      // Input: _signExtendInput (16-bit immediate from instruction), Output: _latchIDEX.signExtImmediate (32-bit sign-extended)
      _signExtend = new SignExtend<16, 32>("SignExtendUnit", &_signExtendInput, &_latchIDEX_next.signExtImmediate);

      // EX Stage,.xxxxxxxxxxxxxxxxxxxxvzc Components
      // MUX for Register Destination (RegDst)
      // Inputs: _latchIDEX.rt, _latchIDEX.rd, Select: _latchIDEX.ctrlEX.regDst, Output: _latchEXMEM.regDstIdx
      _muxRegDst = new MUX2<5>("EX_Mux_RegDst", &_latchIDEX.rt, &_latchIDEX.rd, &_latchIDEX.ctrlEX.regDst, &_latchEXMEM_next.regDstIdx);
      
      // MUX for ALU Source (ALUSrc)
      #ifdef ENABLE_DATA_FORWARDING
      _muxALUSrc = new MUX2<32>("EX_Mux_ALUSrc", &_muxForwardBOutput, &_latchIDEX.signExtImmediate, &_latchIDEX.ctrlEX.aluSrc, &_muxALUSrcOutput);
      #else
      // Inputs: _latchIDEX.regFileReadData2, _latchIDEX.signExtImmediate, Select: _latchIDEX.ctrlEX.aluSrc, Output: _muxALUSrcOutput
      _muxALUSrc = new MUX2<32>("EX_Mux_ALUSrc", &_latchIDEX.regFileReadData2, &_latchIDEX.signExtImmediate, &_latchIDEX.ctrlEX.aluSrc, &_muxALUSrcOutput);
      #endif
      // ALU Control Unit
      // Inputs: _latchIDEX.ctrlEX.aluOp, _aluControlInput (funct field from instruction set in ID), Output: _aluControlOutput
      _aluControl = new ALUControl(&_latchIDEX.ctrlEX.aluOp, &_aluControlInput, &_aluControlOutput);
      
      // ALU
      #ifdef ENABLE_DATA_FORWARDING
      _alu = new ALU(&_aluControlOutput, &_muxForwardAOutput, &_muxALUSrcOutput, &_latchEXMEM_next.aluResult, &_latchEXMEM_next.aluZero);
      #else
      // Inputs: _latchIDEX.regFileReadData1, _muxALUSrcOutput, Control: _aluControlOutput, Outputs: _latchEXMEM.aluResult, __latchEXMEM.aluZero
      _alu = new ALU(&_aluControlOutput, &_latchIDEX.regFileReadData1, &_muxALUSrcOutput, &_latchEXMEM_next.aluResult, &_latchEXMEM_next.aluZero);
      #endif
      // Adder for Branch Target Address
      // Inputs: _latchIDEX.pcPlus4, _adderBranchTargetAddrInput1 (shifted immediate), Output: _latchEXMEM.branchTargetAddr
      _adderBranchTargetAddr = new Adder<32>("EX_Adder_BranchTarget", &_latchIDEX.pcPlus4, &_adderBranchTargetAddrInput1, &_latchEXMEM_next.branchTargetAddr);

      // MEM Stage Components
      // Data Memory
      // Inputs: _latchEXMEM.aluResult (address), _latchEXMEM.regFileReadData2 (writeData for sw)
      //         _latchEXMEM.ctrlMEM.memRead, _latchEXMEM.ctrlMEM.memWrite
      // Output: _wDataMemReadData
      _dataMemory = new Memory("DataMemory", &_latchEXMEM.aluResult, &_latchEXMEM.regFileReadData2, &_latchEXMEM.ctrlMEM.memRead, &_latchEXMEM.ctrlMEM.memWrite, &_latchMEMWB_next.dataMemReadData, memoryEndianness, dataMemFileName);
      
      // MUX for PC Source (PCSrc) - determines next PC value (branch vs. PC+4)
      // Inputs: _pcPlus4 (from IF stage, will be updated each cycle), _wBranchTargetAddr (from EX stage, via EXMEM latch then to MEM stage MUX input)
      // Select: _muxPCSrcSelect (determined in MEM stage), Output: _PC
      _muxPCSrc = new MUX2<32>("MEM_Mux_PCSrc", &_pcPlus4, &_latchEXMEM.branchTargetAddr, &_muxPCSrcSelect, &_PC);

      // WB Stage Components
      // MUX for Memory to Register (MemToReg) - selects data to write back to register file
      // Inputs: _latchMEMWB.aluResult, _latchMEMWB.dataMemReadData, Select: _latchMEMWB.ctrlWB.memToReg, Output: _muxMemToRegOutput
      _muxMemToReg = new MUX2<32>("WB_Mux_MemToReg", &_latchMEMWB.aluResult, &_latchMEMWB.dataMemReadData, &_latchMEMWB.ctrlWB.memToReg, &_muxMemToRegOutput);

      
      #ifdef ENABLE_DATA_FORWARDING
      // ForwardingUnit
      _forwardingUnit = new ForwardingUnit("ForwardingUnit",
                                         &_latchIDEX.rs, &_latchIDEX.rt,
                                         &_latchEXMEM.ctrlWB.regWrite, &_latchEXMEM.regDstIdx,
                                         &_latchMEMWB.ctrlWB.regWrite, &_latchMEMWB.regDstIdx,
                                         &_forwardA, &_forwardB);
      _muxForwardA = new MUX3<32>("Mux_ForwardA",
                                 &_latchIDEX.regFileReadData1,    // Sel 00
                                 &_muxMemToRegOutput,             // Sel 01 (MEM/WB stage output, handles lw vs alu op)
                                 &_latchEXMEM.aluResult,          // Sel 10
                                 &_forwardA, &_muxForwardAOutput);
      _muxForwardB = new MUX3<32>("Mux_ForwardB",
                                 &_latchIDEX.regFileReadData2,    // Sel 00,               // Sel 00 (Output of ALUSrc MUX)
                                 &_muxMemToRegOutput,             // Sel 01
                                 &_latchEXMEM.aluResult,          // Sel 10
                                 &_forwardB, &_muxForwardBOutput);
      #ifdef ENABLE_HAZARD_DETECTION
      _hazDetUnit = new HazardDetectionUnit("HazardDetectionUnit",
                                          &_hazDetIFIDRs,           // Input: rs from IF/ID instruction (set in ID stage)
                                          &_hazDetIFIDRt,           // Input: rt from IF/ID instruction (set in ID stage)
                                          &_latchIDEX.ctrlMEM.memRead, // Input: MemRead from ID/EX latch (current state)
                                          &_latchIDEX.rt,           // Input: rt (potential dest of lw) from ID/EX latch (current state)
                                          &_hazDetPCWrite,          // Output
                                          &_hazDetIFIDWrite,        // Output
                                          &_hazDetIDEXCtrlWrite);   // Output
      #endif
      #endif //ENABLE_DATA_FORWARDING
    }

    virtual void advanceCycle() {
      /* FIXME: implement the per-cycle behavior of the five-stage pipelined MIPS CPU */

      // WB Stage
      // 1. Determine data to write to register file using MUX_MemToReg
      _muxMemToReg->advanceCycle();

      // MEM Stage
      // 1. Data Memory Access (if needed)
      _dataMemory->advanceCycle();
      #ifdef ENABLE_DATA_FORWARDING
      uint32_t pc_val_before_mux_update = _PC.to_ulong();
      #endif
      
      // 2. Determine PCSrc select signal for MUX_PCSrc (for branch resolution)
      _muxPCSrcSelect = _latchEXMEM.ctrlMEM.branch &_latchEXMEM.aluZero;


      _latchMEMWB_next.ctrlWB = _latchEXMEM.ctrlWB;
      _latchMEMWB_next.aluResult = _latchEXMEM.aluResult;
      _latchMEMWB_next.regDstIdx = _latchEXMEM.regDstIdx;


      // EX Stage
      #ifdef ENABLE_DATA_FORWARDING
      // 0. Determine forwarding singnals
      _forwardingUnit->advanceCycle();
      #endif //ENABLE_DATA_FORWARDING

      // 1. MUX_RegDst : Selects destination register
      _muxRegDst->advanceCycle();

      // 2. ALU control
      _aluControlInput = _latchIDEX.signExtImmediate.to_ulong() & 0x3F;
      _aluControl->advanceCycle();
      #ifdef ENABLE_DATA_FORWARDING
      _muxForwardA->advanceCycle(); // _muxForwardAOutput
      _muxForwardB->advanceCycle(); // _muxForwardBoutput
      #endif //ENABLE_DATA_FORWARDING
      // 3. MUX_ALUSrc : Selects source for ALU
      _muxALUSrc->advanceCycle();
      // 4. alu
      _alu->advanceCycle();

      // 5. Adder_BranchTargetAddr
      _adderBranchTargetAddrInput1 = (_latchIDEX.signExtImmediate.to_ulong() << 2);
      _adderBranchTargetAddr->advanceCycle();

      _latchEXMEM_next.ctrlWB = _latchIDEX.ctrlWB;
      _latchEXMEM_next.ctrlMEM = _latchIDEX.ctrlMEM;
      _latchEXMEM_next.regFileReadData2 = _latchIDEX.regFileReadData2;


      // ID
      // 1. Decode instruction fields from _latchIFID.instruction
      _opcode = (_latchIFID.instruction.to_ulong() >> 26) & 0x3F;
      _regFileReadRegister1 = (_latchIFID.instruction.to_ulong() >> 21) & 0x1F;
      _regFileReadRegister2 = (_latchIFID.instruction.to_ulong() >> 16) & 0x1F;
      _signExtendInput = _latchIFID.instruction.to_ulong() & 0xFFFF;

      #ifdef ENABLE_HAZARD_DETECTION
      _hazDetIFIDRs = (_latchIFID.instruction.to_ulong() >> 21) & 0x1F;
      _hazDetIFIDRt = (_latchIFID.instruction.to_ulong() >> 16) & 0x1F;
      _hazDetUnit->advanceCycle();
      #endif
      // 2. Control Unit
      _control->advanceCycle();

      // 3. Sign Extend Unit
      _signExtend->advanceCycle();

      #ifdef ENABLE_HAZARD_DETECTION
      if (_hazDetIDEXCtrlWrite.test(0) == 0) { // If IDEXCtrlWrite is 0, insert bubble
        _latchIDEX_next.ctrlWB.memToReg = 0;
        _latchIDEX_next.ctrlWB.regWrite = 0;
        _latchIDEX_next.ctrlMEM.branch = 0;
        _latchIDEX_next.ctrlMEM.memRead = 0;
        _latchIDEX_next.ctrlMEM.memWrite = 0;
        _latchIDEX_next.ctrlEX.regDst = 0;
        _latchIDEX_next.ctrlEX.aluOp = 0;
        _latchIDEX_next.ctrlEX.aluSrc = 0;
      }
      #endif //ENABLE_HAZARD_DETECTION
      

      // 4. Populate 'latchIDEX'
      _latchIDEX_next.pcPlus4 = _latchIFID.pcPlus4;

      _latchIDEX_next.rt = (_latchIFID.instruction.to_ulong() >> 16) & 0x1F;
      _latchIDEX_next.rd = (_latchIFID.instruction.to_ulong() >> 11) & 0x1F;
      #ifdef ENABLE_DATA_FORWARDING
      _latchIDEX_next.rs = (_latchIFID.instruction.to_ulong() >> 21) & 0x1F;
      #endif

      _registerFile->advanceCycle();

      // ----- IF -----
      // 1. Adder_PCPlus4 : Calculate PC + 4
      _adderPCPlus4->advanceCycle();
      _muxPCSrc->advanceCycle();
      #ifdef ENABLE_HAZARD_DETECTION
      if (_hazDetPCWrite.test(0) == 0){
        _PC = pc_val_before_mux_update;
      }
      #endif
      // 2. Instruction Memory Access
      _instMemory->advanceCycle();
      _adderPCPlus4->advanceCycle(); 
      
      // 3. Populate 'latchIFID_next'
      #ifdef ENABLE_HAZARD_DETECTION
      if (_hazDetIFIDWrite.test(0)) { // If IFIDWrite is 1, update IF/ID latch normally
        _latchIFID_next.pcPlus4 = _pcPlus4; // _pcPlus4는 _adderPCPlus4의 출력 와이어
        // _latchIFID_next.instruction는 생성자에서 _instMemory 출력에 이미 연결됨
      } else { // If IFIDWrite is 0, stall the IF/ID latch (keep current values)
        _latchIFID_next = _latchIFID;
      }
      #else // No hazard detection, always update normally
      _latchIFID_next.pcPlus4 = _pcPlus4;
      // _latchIFID_next.instruction는 생성자에서 _instMemory 출력에 이미 연결됨
       #endif

      // 모든 계산이 끝난 후, "다음 상태" 래치들을 "현재 상태" 래치로 복사
      _latchIFID = _latchIFID_next;
      _latchIDEX = _latchIDEX_next;
      _latchEXMEM = _latchEXMEM_next;
      _latchMEMWB = _latchMEMWB_next;
      
      // increase Cycle
      _currCycle += 1;
    }

    ~PipelinedCPU() {
      delete _adderPCPlus4;
      delete _instMemory;
      delete _control;
      delete _registerFile;
      delete _signExtend;
      delete _adderBranchTargetAddr;
      delete _muxALUSrc;
      delete _aluControl;
      delete _alu;
      delete _muxRegDst;
      delete _muxPCSrc;
      delete _dataMemory;
      delete _muxMemToReg;
#ifdef ENABLE_DATA_FORWARDING
      delete _forwardingUnit;
      delete _muxForwardA;
      delete _muxForwardB;
#ifdef ENABLE_HAZARD_DETECTION
      delete _hazDetUnit;
#endif
#endif
    }

  private:

    // Cycle tracker
    std::uint64_t _currCycle = 0;

    // Always-1/0 wires
    const Wire<1> _alwaysHi = 1;
    const Wire<1> _alwaysLo = 0;

    // Components for the IF stage
    Register<32> _PC; // the Program Counter (PC) register
    Adder<32> *_adderPCPlus4; // the 32-bit adder in the IF stage
    Memory *_instMemory; // the instruction memory
    // Components for the ID stage
    Control *_control; // the Control unit
    RegisterFile *_registerFile; // the Register File
    SignExtend<16, 32> *_signExtend; // the sign-extend unit
    // Components for the EX stage
    Adder<32> *_adderBranchTargetAddr; // the 32-bit adder in the EX stage
    MUX2<32> *_muxALUSrc; // the MUX whose control signal is 'ALUSrc'
    ALUControl *_aluControl; // the ALU Control unit
    ALU *_alu; // the ALU
    MUX2<5> *_muxRegDst; // the MUX whose control signal is 'RegDst'
    // Components for the MEM stage
    MUX2<32> *_muxPCSrc; // the MUX whose control signal is 'PCSrc'
    Memory *_dataMemory; // the data memory
    // Components for the WB stage
    MUX2<32> *_muxMemToReg; // the MUX whose control signal is 'MemToReg'
#ifdef ENABLE_DATA_FORWARDING
    ForwardingUnit *_forwardingUnit; // the forwarding unit
    MUX3<32> *_muxForwardA; // the 3-to-1 MUX whose control signal is 'forwardA'
    MUX3<32> *_muxForwardB; // the 3-to-1 MUX whose control signal is 'forwardB'
#ifdef ENABLE_HAZARD_DETECTION
    HazardDetectionUnit *_hazDetUnit; // the Hazard Detection unit
#endif
#endif

    // Latches
    typedef struct {
      Register<1> regDst;
      Register<2> aluOp;
      Register<1> aluSrc;
    } ControlEX_t; // the control signals for the EX stage
    typedef struct {
      Register<1> branch;
      Register<1> memRead;
      Register<1> memWrite;
    } ControlMEM_t; // the control signals for the MEM stage
    typedef struct {
      Register<1> memToReg;
      Register<1> regWrite;
    } ControlWB_t; // the control signals for the WB stage
    struct {
      Register<32> pcPlus4; // PC+4
      Register<32> instruction; // 32-bit instruction
    } _latchIFID = {}; // the IF-ID latch
    struct {
      ControlWB_t ctrlWB; // the control signals for the WB stage
      ControlMEM_t ctrlMEM; // the control signals for the MEM stage
      ControlEX_t ctrlEX; // the control signals for the EX stage
      Register<32> pcPlus4; // PC+4
      Register<32> regFileReadData1; // 'ReadData1' from the register file
      Register<32> regFileReadData2; // 'ReadData2' from the register file
      Register<32> signExtImmediate; // the 32-bit sign-extended immediate value
#ifdef ENABLE_DATA_FORWARDING
      Register<5> rs; // the 5-bit 'rs' field
#endif
      Register<5> rt; // the 5-bit 'rt' field
      Register<5> rd; // the 5-bit 'rd' field
    } _latchIDEX = {}; // the ID-EX latch
    struct {
      ControlWB_t ctrlWB; // the control signals for the WB stage
      ControlMEM_t ctrlMEM; // the control signals for the MEM stage
      Register<32> branchTargetAddr; // the 32-bit branch target address
      Register<1> aluZero; // 'Zero' from the ALU
      Register<32> aluResult; // the 32-bit ALU output
      Register<32> regFileReadData2; // 'ReadData2' from the register file
      Register<5> regDstIdx; // the index of the destination register
    } _latchEXMEM = {}; // the EX-MEM latch
    struct {
      ControlWB_t ctrlWB; // the control signals for the WB stage
      Register<32> dataMemReadData; // the 32-bit data read from the data memory
      Register<32> aluResult; // the 32-bit ALU output
      Register<5> regDstIdx; // the index of the destination register
    } _latchMEMWB = {}; // the MEM-WB latch

    decltype(_latchIFID) _latchIFID_next;
    decltype(_latchIDEX) _latchIDEX_next;
    decltype(_latchEXMEM) _latchEXMEM_next;
    decltype(_latchMEMWB) _latchMEMWB_next;

    // Wires
    Wire<32> _adderPCPlus4Input1; // the second input to the adder in the IF stage (i.e., 4)
    Wire<32> _pcPlus4; // the output of the adder in the IF stage
    Wire<6> _opcode; // the input to the Control unit
    Wire<5> _regFileReadRegister1; // 'ReadRegister1' for the Register File
    Wire<5> _regFileReadRegister2; // 'ReadRegister2' for the Register File
    Wire<32> _muxMemToRegOutput; // the output of the MUX whose control signal is 'MemToReg'
    Wire<16> _signExtendInput; // the input to the sign-extend unit
    Wire<32> _adderBranchTargetAddrInput1; // the second input to the adder in the EX stage
    Wire<32> _muxALUSrcOutput; // the output of the MUX whose control signal is 'ALUSrc'
    Wire<6> _aluControlInput; // the input to the ALU Control unit (i.e., the 'funct' field)
    Wire<4> _aluControlOutput; // the output of the ALU Control unit
    Wire<1> _muxPCSrcSelect; // the control signal (a.k.a. selector) for the MUX whose control signal is 'PCSrc'

#ifdef ENABLE_DATA_FORWARDING
    Wire<2> _forwardA, _forwardB; // the outputs from the Forwarding unit
    Wire<32> _muxForwardAOutput; // the output of the 3-to-1 MUX whose control signal is 'forwardA'
    Wire<32> _muxForwardBOutput; // the output of the 3-to-1 MUX whose control signal is 'forwardB'
#ifdef ENABLE_HAZARD_DETECTION
    Wire<5> _hazDetIFIDRs, _hazDetIFIDRt; // the inputs to the Hazard Detection unit
    Wire<1> _hazDetPCWrite, _hazDetIFIDWrite, _hazDetIDEXCtrlWrite; // the outputs of the Hazard Detection unit
#endif
#endif

  public:

    void printPVS() {
      printf("==================== Cycle %lu ====================\n", _currCycle);
      printf("PC = 0x%08lx\n", _PC.to_ulong());
      printf("Registers:\n");
      _registerFile->printRegisters();
      printf("Data Memory:\n");
      _dataMemory->printMemory();
      printf("Instruction Memory:\n");
      _instMemory->printMemory();
      printf("Latches:\n");
      printf("  IF-ID Latch:\n");
      printf("    pcPlus4          = 0x%08lx\n", _latchIFID.pcPlus4.to_ulong());
      printf("    instruction      = 0x%08lx\n", _latchIFID.instruction.to_ulong());
      printf("  ID-EX Latch:\n");
      printf("    ctrlWBMemToReg   = 0b%s\n", _latchIDEX.ctrlWB.memToReg.to_string().c_str());
      printf("    ctrlWBRegWrite   = 0b%s\n", _latchIDEX.ctrlWB.regWrite.to_string().c_str());
      printf("    ctrlMEMBranch    = 0b%s\n", _latchIDEX.ctrlMEM.branch.to_string().c_str());
      printf("    ctrlMEMMemRead   = 0b%s\n", _latchIDEX.ctrlMEM.memRead.to_string().c_str());
      printf("    ctrlMEMMemWrite  = 0b%s\n", _latchIDEX.ctrlMEM.memWrite.to_string().c_str());
      printf("    ctrlEXRegDst     = 0b%s\n", _latchIDEX.ctrlEX.regDst.to_string().c_str());
      printf("    ctrlEXALUOp      = 0b%s\n", _latchIDEX.ctrlEX.aluOp.to_string().c_str());
      printf("    ctrlEXALUSrc     = 0b%s\n", _latchIDEX.ctrlEX.aluSrc.to_string().c_str());
      printf("    pcPlus4          = 0x%08lx\n", _latchIDEX.pcPlus4.to_ulong());
      printf("    regFileReadData1 = 0x%08lx\n", _latchIDEX.regFileReadData1.to_ulong());
      printf("    regFileReadData2 = 0x%08lx\n", _latchIDEX.regFileReadData2.to_ulong());
      printf("    signExtImmediate = 0x%08lx\n", _latchIDEX.signExtImmediate.to_ulong());
#ifdef ENABLE_DATA_FORWARDING
      printf("    rs               = 0b%s\n", _latchIDEX.rs.to_string().c_str());
#endif
      printf("    rt               = 0b%s\n", _latchIDEX.rt.to_string().c_str());
      printf("    rd               = 0b%s\n", _latchIDEX.rd.to_string().c_str());
      printf("  EX-MEM Latch:\n"); 
      printf("    ctrlWBMemToReg   = 0b%s\n", _latchEXMEM.ctrlWB.memToReg.to_string().c_str());
      printf("    ctrlWBRegWrite   = 0b%s\n", _latchEXMEM.ctrlWB.regWrite.to_string().c_str());
      printf("    ctrlMEMBranch    = 0b%s\n", _latchEXMEM.ctrlMEM.branch.to_string().c_str());
      printf("    ctrlMEMMemRead   = 0b%s\n", _latchEXMEM.ctrlMEM.memRead.to_string().c_str());
      printf("    ctrlMEMMemWrite  = 0b%s\n", _latchEXMEM.ctrlMEM.memWrite.to_string().c_str());
      printf("    branchTargetAddr = 0x%08lx\n", _latchEXMEM.branchTargetAddr.to_ulong());
      printf("    aluZero          = 0b%s\n", _latchEXMEM.aluZero.to_string().c_str());
      printf("    aluResult        = 0x%08lx\n", _latchEXMEM.aluResult.to_ulong());
      printf("    regFileReadData2 = 0x%08lx\n", _latchEXMEM.regFileReadData2.to_ulong());
      printf("    regDstIdx        = 0b%s\n", _latchEXMEM.regDstIdx.to_string().c_str());
      printf("  MEM-WB Latch:\n"); 
      printf("    ctrlWBMemToReg   = 0b%s\n", _latchMEMWB.ctrlWB.memToReg.to_string().c_str());
      printf("    ctrlWBRegWrite   = 0b%s\n", _latchMEMWB.ctrlWB.regWrite.to_string().c_str());
      printf("    dataMemReadData  = 0x%08lx\n", _latchMEMWB.dataMemReadData.to_ulong());
      printf("    aluResult        = 0x%08lx\n", _latchMEMWB.aluResult.to_ulong());
      printf("    regDstIdx        = 0b%s\n", _latchMEMWB.regDstIdx.to_string().c_str());
    }

};

#endif

