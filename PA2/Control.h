#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "DigitalCircuit.h"

class Control : public DigitalCircuit {

  public:

    Control(const Wire<6> *iOpcode,
            Wire<1> *oRegDst,
            Wire<1> *oALUSrc,
            Wire<1> *oMemToReg,
            Wire<1> *oRegWrite,
            Wire<1> *oMemRead,
            Wire<1> *oMemWrite,
            Wire<1> *oBranch,
            Wire<2> *oALUOp) : DigitalCircuit("Control") {
      _iOpcode = iOpcode;
      _oRegDst = oRegDst;
      _oALUSrc = oALUSrc;
      _oMemToReg = oMemToReg;
      _oRegWrite = oRegWrite;
      _oMemRead = oMemRead;
      _oMemWrite = oMemWrite;
      _oBranch = oBranch;
      _oALUOp = oALUOp;
    }

    virtual void advanceCycle() {
      unsigned long opcode_val = _iOpcode->to_ulong();

      // initialize
      *_oRegDst = 0;
      *_oALUSrc = 0;
      *_oMemToReg = 0;
      *_oRegWrite = 0;
      *_oMemRead = 0;
      *_oMemWrite = 0;
      *_oBranch = 0;
      *_oALUOp = 0;

      // 제어 신호 설정
      switch (opcode_val) {
        case 0x00: // R-format
          // Opcode: 000000
          *_oRegDst = 1;
          *_oRegWrite = 1;
          *_oALUOp = 0x2; // ALUOp = 10
          break;

        case 0x23: // lw (load word)
          // Opcode: 100011
          *_oALUSrc = 1;
          *_oMemToReg = 1;
          *_oRegWrite = 1;
          *_oMemRead = 1;
          // _oALUOp는 00 (덧셈)으로 유지 (초기화 값 사용)
          break;

        case 0x2b: // sw (store word)
          // Opcode: 101011
          *_oALUSrc = 1;
          *_oMemWrite = 1;
          // _oALUOp는 00 (덧셈)으로 유지 (초기화 값 사용)
          // RegDst, MemtoReg, RegWrite, MemRead, Branch는 0으로 유지
          break;

        case 0x04: // beq (branch on equal)
          // Opcode: 000100
          *_oBranch = 1;
          *_oALUOp = 0x1; // ALUOp = 01 (뺄셈)
          // RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite는 0으로 유지
          break;

        default:
          // 명시적으로 처리되지 않은 opcode의 경우, 모든 제어 신호는 초기값 0을 유지합니다.
          // 이는 "addi"와 같이 표에 명시되지 않은 명령어에 대한 처리 방침과도 부합합니다.
          // (모든 신호 0은 안전한 "no-operation"에 가까운 상태를 유도할 수 있습니다)
          break;
      }
    }

  private:

    const Wire<6> *_iOpcode;
    Wire<1> *_oRegDst;
    Wire<1> *_oALUSrc;
    Wire<1> *_oMemToReg;
    Wire<1> *_oRegWrite;
    Wire<1> *_oMemRead;
    Wire<1> *_oMemWrite;
    Wire<1> *_oBranch;
    Wire<2> *_oALUOp;

};

#endif

