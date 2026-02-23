#ifndef __SINGLE_CYCLE_CPU_H__
#define __SINGLE_CYCLE_CPU_H__

#include "DigitalCircuit.h"

#include "ALU.h"
#include "ALUControl.h"
#include "Control.h"
#include "RegisterFile.h"
#include "Memory.h"

template<size_t N>
class MUX : public DigitalCircuit {

  public:

    MUX(const std::string &name,
        const Wire<N> *iInput0,
        const Wire<N> *iInput1,
        const Wire<1> *iSelect,
        Wire<N> *oOutput) : DigitalCircuit(name) {
      _iInput0 = iInput0;
      _iInput1 = iInput1;
      _iSelect = iSelect;
      _oOutput = oOutput;
    }

    virtual void advanceCycle() {
      if (_iSelect->test(0)) {
        *_oOutput = *_iInput1;
      } else {
        *_oOutput = *_iInput0;
      }
    }

  private:

    const Wire<N> *_iInput0;
    const Wire<N> *_iInput1;
    const Wire<1> *_iSelect;
    Wire<N> *_oOutput;

};

class SingleCycleCPU : public DigitalCircuit {

  public:

    SingleCycleCPU(const std::string name,
                   const std::uint32_t initialPC,
                   const char *regFileName,
                   const char *instMemFileName,
                   const char *dataMemFileName) : DigitalCircuit(name) {
      _currCycle = 0;

      _alwaysHi = 1;
      _alwaysLo = 0;

      _PC = initialPC;

      _dummyWriteDataForInstMem.reset();

      // 1. 명령어 메모리 (Instruction Memory)
      // 주소 입력 : _PC
      // 쓰기 데이터입력: _dummyWriteDataForInstMem (사용 안 함)
      // MemRead: _alwaysHi (항상 읽기)
      // MemWrite : _alwaysLo (쓰기 안 함)
      // 읽은 데이터출력 : _instMemInstruction
      _instMemory = new Memory("InsetMemory", &_PC, &_dummyWriteDataForInstMem, &_alwaysHi, &_alwaysLo, &_instMemInstruction, Memory::LittleEndian, instMemFileName);

      // 2. 레지스터 파일 (Register File)
      _registerFile = new RegisterFile(&_regFileReadRegister1, &_regFileReadRegister2, &_muxRegFileWriteRegisterOutput, &_regFileWriteData, &_ctrlRegWrite, &_regFileReadData1, &_regFileReadData2, regFileName);

      // 3. 데이터 메모리 (Data Memory)
      _dataMemory = new Memory("DataMemory", &_aluResult, &_regFileReadData2 /* SW 명령어의 WriteData */, &_ctrlMemRead, &_ctrlMemWrite, &_dataMemReadData, Memory::LittleEndian, dataMemFileName);

      // 4. 제어 유닛들 (Control Units)
      _control = new Control(&_ctrlOpcode, &_ctrlRegDst, &_ctrlALUSrc, &_ctrlMemToReg, &_ctrlRegWrite, &_ctrlMemRead, &_ctrlMemWrite, &_ctrlBranch, &_ctrlALUOp);
      _aluControl = new ALUControl(&_ctrlALUOp, &_aluCtrlFunct, &_aluCtrlOp);
      _alu = new ALU(&_aluCtrlOp, &_regFileReadData1, &_muxALUInput1Output, &_aluResult, &_aluZero);

      // 5. MUX들 (Multiplexers)
      _muxRegFileWriteRegister = new MUX<5>("Mux_RegDst", &_muxRegFileWriteRegisterInput0, &_muxRegFileWriteRegisterInput1, &_ctrlRegDst, &_muxRegFileWriteRegisterOutput);
      _muxALUInput1 = new MUX<32>("Mux_ALUSrc", &_regFileReadData2, &_signExtendOutput, &_ctrlALUSrc, &_muxALUInput1Output);
      _muxRegFileWriteData = new MUX<32>("Mux_MemToReg", &_aluResult, &_dataMemReadData, &_ctrlMemToReg, &_regFileWriteData);
      _muxPC = new MUX<32>("Mux_PCSrc", &_muxPCInput0, &_muxPCInput1, &_muxPCSelect, &_muxPCOutput);
    }

    void printPVS() {
      printf("==================== Cycle %lu ====================\n", _currCycle);
      printf("PC = 0x%08lx\n", _PC.to_ulong());
      printf("Registers:\n");
      _registerFile->printRegisters();
      printf("Data Memory:\n");
      _dataMemory->printMemory();
      printf("Instruction Memory:\n");
      _instMemory->printMemory();
    }

    virtual void advanceCycle() {
      _currCycle += 1;

      /* FIXME: implement the single-cycle behavior of the single-cycle MIPS CPU */

      // ----------------------------------------------------------------------------
      // STAGE 1: Instruction Fetch (IF)

      // 명령어 메모리는 생성자에서 _PC를 주소로, _alwaysHi를 MemRead로 연결

      // _PC 주소의 명령어를 읽어 _instMemInstruction에 저장
      _instMemory->advanceCycle();


      // ----------------------------------------------------------------------------
      // STAGE 2 : Instruction Decode & Register Fetch (ID)
      // 명령어 필드 추출
      _ctrlOpcode = (_instMemInstruction.to_ulong() >> 26) & 0x3F; // opcode: inst[31-26]
      _regFileReadRegister1 = (_instMemInstruction.to_ulong() >> 21) & 0x1F; // rs : inst[25-21]
      _regFileReadRegister2 = (_instMemInstruction.to_ulong() >> 16) & 0x1F; // rt : inst[20-16]
      _muxRegFileWriteRegisterInput0 = (_instMemInstruction.to_ulong() >> 16) & 0x1F; // MUX RegDst 입력 0 : rt for lw
      _muxRegFileWriteRegisterInput1 = (_instMemInstruction.to_ulong() >> 11) & 0x1F; // MUS RegDst 입력 1 : rd for R-type
      _signExtendInput = _instMemInstruction.to_ulong() & 0xFFFF; // Immediate : inst[15-0]
      _aluCtrlFunct = _instMemInstruction.to_ulong() & 0x3F; // Funct : inst[5-0]

      // 제어 유닛 (_control) 동작
      // 입력 : ctrlOpcode
      // 출력: _ctrlRegDst, _ctrlALUSrc, _ctrlMemToReg, _ctrlRegWrite, _ctrlMemRead, _ctrlMemWrite, _ctrlBranch, _ctrlALUOp
      _control->advanceCycle();

      // ALU 제어 유닛 (_aluControl) 동작 - ID 단계에서 ALUOp와 Funct가 결정되므로 여기서 호출
      // 입력: _ctrlALUOp (Control 유닛 출력), _aluCtrlFunct (명령어에서 추출)
      // 출력: _aluCtrlOp (ALU 연산 종류)
      _aluControl->advanceCycle();

      // 레지스터 파일 (_registerFile) 읽기 (쓰기 방지)
      // 읽을 레지스터 번호 : _regFileReadRegister1(rs), _regFileReadRegister2(rt) (이미 설정됨)
      // 출력 : _regFileReadData1, _regFileReadData2

      Wire<1> originalCtrlRegWrite = _ctrlRegWrite; // 원래 _ctrlRegWrite 값 저장
      _ctrlRegWrite = 0; // 읽기만 수행하도록 RegWrite를 0으로 임시 설정
      _registerFile->advanceCycle(); // 읽기 수행
      _ctrlRegWrite = originalCtrlRegWrite; // 원래 _ctrlRegWrite 값 복원


      // ----------------------------------------------------------------------------
      // STAGE 3 : Execute (EX)
      // 부호 확장 (SignExtension) - 16비트 즉시값을 32비트로 부호 확장
      if (_signExtendInput.test(15)) { // 16비트 즉시값의 최상위 비트(부호 비트)가 1이면
        _signExtendOutput = 0xFFFF0000 | _signExtendInput.to_ulong(); // 음수이므로 상위 16비트를 1로 채움
      } else {
        _signExtendOutput = _signExtendInput.to_ulong(); // 양수이므로 상위 16비트를 0으로 채움
      }

      // ALU 입력 MUX (_muxALUInput1) 동작 (ALUSrc MUX)
      // 입력 0: _regFileReadData2 (rt 값)
      // 입력 1: _signExtendOutput (부호 확장된 즉시값)
      // 선택 신호: _ctrlALUSrc (Control 유닛 출력)
      // 출력: _muxALUInput1Output
      _muxALUInput1->advanceCycle();

      // ALU (_alu) 동작
      // 입력 0: _regFileReadData1 (rs 값)
      // 입력 1: _muxALUInput1Output (ALUSrc MUX의 출력)
      // 제어 신호: _aluCtrlOp (ALUControl 유닛 출력)
      // 출력: _aluResult (연산 결과), _aluZero (결과가 0인지 여부)
      _alu->advanceCycle();

      // --------------------------------------------------------------------
      // STAGE 4: Memory Access (MEM)
      // --------------------------------------------------------------------
      // 데이터 메모리 (_dataMemory) 접근
      // 주소: _aluResult (ALU 연산 결과)
      // 쓸 데이터 (sw의 경우): _regFileReadData2 (rt 값)
      // 제어 신호: _ctrlMemRead, _ctrlMemWrite (Control 유닛 출력)
      // 출력 (lw의 경우): _dataMemReadData
      _dataMemory->advanceCycle();

      /// --------------------------------------------------------------------
      // STAGE 5: Write Back (WB)
      // --------------------------------------------------------------------
      // 레지스터에 쓸 데이터 선택 MUX (_muxRegFileWriteData) 동작 (MemToReg MUX)
      // 입력 0: _aluResult (ALU 연산 결과)
      // 입력 1: _dataMemReadData (데이터 메모리에서 읽은 값)
      // 선택 신호: _ctrlMemToReg (Control 유닛 출력)
      // 출력: _regFileWriteData (레지스터 파일에 쓸 최종 데이터)
      _muxRegFileWriteData->advanceCycle();

      // 목적지 레지스터 선택 MUX (_muxRegFileWriteRegister) 동작 (RegDst MUX)
      // 입력 0: _muxRegFileWriteRegisterInput0 (rt 번호)
      // 입력 1: _muxRegFileWriteRegisterInput1 (rd 번호)
      // 선택 신호: _ctrlRegDst (Control 유닛 출력)
      // 출력: _muxRegFileWriteRegisterOutput (레지스터 파일에 쓸 목적지 레지스터 번호)
      _muxRegFileWriteRegister->advanceCycle();

      // 실제 레지스터 파일 쓰기 동작
      // 이 시점에는 _muxRegFileWriteRegisterOutput, _regFileWriteData, _ctrlRegWrite 와이어에
      // 현재 명령어에 대한 최종 값이 모두 설정되어 있음.
      _registerFile->advanceCycle(); // 최종 값으로 쓰기 수행 (만약 _ctrlRegWrite가 1이라면)

      // --------------------------------------------------------------------
      // PC Update
      // --------------------------------------------------------------------
      // PC + 4 계산
      _muxPCInput0 = _PC.to_ulong() + 4;

      // 분기 주소 계산: (PC + 4) + (sign_extended_immediate << 2)
      Wire<32> branch_offset_shifted = _signExtendOutput.to_ulong() << 2; // 즉시값을 왼쪽으로 2비트 시프트
      _muxPCInput1 = (_PC.to_ulong() + 4) + branch_offset_shifted.to_ulong();

      // PCSrc 신호 계산: Branch 제어 신호가 1이고 ALU의 Zero 출력이 1이면 분기 (PCSrc = 1)
      _muxPCSelect = _ctrlBranch.test(0) && _aluZero.test(0);

      // 다음 PC 값 선택 MUX (_muxPC) 동작
      // 입력 0: PC + 4 (_muxPCInput0)
      // 입력 1: 분기 목표 주소 (_muxPCInput1)
      // 선택 신호: _muxPCSelect (PCSrc)
      // 출력: _muxPCOutput
      _muxPC->advanceCycle();

      // PC 레지스터 업데이트
      _PC = _muxPCOutput;
    }

    ~SingleCycleCPU() {
      delete _instMemory;
      delete _registerFile;
      delete _dataMemory;

      delete _control;
      delete _aluControl;
      delete _alu;
      delete _muxRegFileWriteRegister;
      delete _muxALUInput1;
      delete _muxRegFileWriteData;
      delete _muxPC;
    }

  private:

    // Cycle tracker
    std::uint64_t _currCycle;

    // Always-1/0 wires
    Wire<1> _alwaysHi;
    Wire<1> _alwaysLo;

    // 명령어 메모리 쓰기 데이터용 dummy wire(항상 0)
    Wire<32> _dummyWriteDataForInstMem;

    // Program Counter (PC) register
    Register<32> _PC;

    // Sequential circuits
    Memory *_instMemory;
    RegisterFile *_registerFile;
    Memory *_dataMemory;

    // Combinational circuits
    Control *_control;
    ALUControl *_aluControl;
    ALU *_alu;
    MUX<5> *_muxRegFileWriteRegister;
    MUX<32> *_muxALUInput1;
    MUX<32> *_muxRegFileWriteData;
    MUX<32> *_muxPC;

    // instruction[31-0] from the instruction memory
    Wire<32> _instMemInstruction;
    // the control signals to/from the Control unit
    Wire<6> _ctrlOpcode;
    Wire<1> _ctrlRegDst, _ctrlALUSrc, _ctrlMemToReg, _ctrlRegWrite, _ctrlMemRead, _ctrlMemWrite, _ctrlBranch;
    Wire<2> _ctrlALUOp;
    // the wires to/from the MUX in front of the Register File's Write Register
    Wire<5> _muxRegFileWriteRegisterInput0, _muxRegFileWriteRegisterInput1, _muxRegFileWriteRegisterOutput;
    // the wires to/from the Register File
    Wire<5> _regFileReadRegister1, _regFileReadRegister2;
    Wire<32> _regFileWriteData, _regFileReadData1, _regFileReadData2;
    // the wires to/from the Sign-extend unit
    Wire<16> _signExtendInput;
    Wire<32> _signExtendOutput;
    // the wires from the MUX in front of the ALU's second input
    Wire<32> _muxALUInput1Output;
    // the wires to/from the ALU control unit
    Wire<6> _aluCtrlFunct;
    Wire<4> _aluCtrlOp;
    // the wires from the ALU
    Wire<32> _aluResult;
    Wire<1> _aluZero;
    // the wire from the data memory
    Wire<32> _dataMemReadData;
    // the wires to/from the MUX in front of the PC register
    Wire<32> _muxPCInput0, _muxPCInput1;
    Wire<1> _muxPCSelect; // a.k.a. PCSrc (Branch && ALUZero)

    Wire<32> _muxPCOutput;

};

#endif

