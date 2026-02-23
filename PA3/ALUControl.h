#ifndef __ALU_CONTROL_H__
#define __ALU_CONTROL_H__

#include "DigitalCircuit.h"

#include <cassert>

class ALUControl : public DigitalCircuit {

  public:

    ALUControl(const Wire<2> *iALUOp,
               const Wire<6> *iFunct,
               Wire<4> *oOperation) : DigitalCircuit("ALUControl") {
      _iALUOp = iALUOp;
      _iFunct = iFunct;
      _oOperation = oOperation;
    }

    virtual void advanceCycle() {
      /* FIXME */
      unsigned long aluOp_val = _iALUOp->to_ulong(); // 2비트 ALUOp 값을 정수로 변환

      if (aluOp_val == 0x0) {
        *_oOperation = 0x2;    // 출력 Operation 신호: 0010 (덧셈)
      } else if (aluOp_val == 0x1) {
        *_oOperation = 0x6;    // 출력 Operation 신호: 0110 (뺄셈)
      } else if (aluOp_val == 0x2) {
        // Funct 필드의 하위 비트들(F3, F2, F1, F0)을 보고 실제 연산 결정
        bool F0 = _iFunct->test(0); // Funct 필드의 0번째 비트 (F0)
        bool F1 = _iFunct->test(1); // Funct 필드의 1번째 비트 (F1)
        bool F2 = _iFunct->test(2); // Funct 필드의 2번째 비트 (F2)
        bool F3 = _iFunct->test(3); // Funct 필드의 3번째 비트 (F3)
        // F5, F4는 이 특정 연산들 구분에는 사용되지 않음 (표에서 X로 표시된 부분)

        if (!F3 && !F2 && !F1 && !F0) { // Funct[3-0] = 0000 (예: add 명령어)
          *_oOperation = 0x2; // Operation: 0010 (덧셈)
        } else if (!F3 && !F2 && F1 && !F0) { // Funct[3-0] = 0010 (예: sub 명령어)
          *_oOperation = 0x6; // Operation: 0110 (뺄셈)
        } else if (!F3 && F2 && !F1 && !F0) { // Funct[3-0] = 0100 (예: and 명령어)
          *_oOperation = 0x0; // Operation: 0000 (AND)
        } else if (!F3 && F2 && !F1 && F0) { // Funct[3-0] = 0101 (예: or 명령어)
          *_oOperation = 0x1; // Operation: 0001 (OR)
        } else if (F3 && !F2 && F1 && !F0) { // Funct[3-0] = 1010 (예: slt 명령어)
          *_oOperation = 0x7; // Operation: 0111 (SLT)
        } else {
          assert(false && "ALUControl: R-타입 명령어에 대해 처리되지 않은 Funct 코드");
        }
      } else {
        assert(false && "ALUControl: 유효하지 않은 ALUOp 값");
      }
    }

  private:

    const Wire<2> *_iALUOp;
    const Wire<6> *_iFunct;
    Wire<4> *_oOperation;

};

#endif

