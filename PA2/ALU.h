#ifndef __ALU_H__
#define __ALU_H__

#include "DigitalCircuit.h"

#include <cassert>
#include <cstdint>

class ALU : public DigitalCircuit {

  public:

    ALU(const Wire<4> *iALUControl,
        const Wire<32> *iInput0,
        const Wire<32> *iInput1,
        Wire<32> *oOutput,
        Wire<1> *oZero) : DigitalCircuit("ALU") {
      _iALUControl = iALUControl;
      _iInput0 = iInput0;
      _iInput1 = iInput1;
      _oOutput = oOutput;
      _oZero = oZero;
    }

    virtual void advanceCycle() {
      _oOutput->reset(); //출력 와이어 초기화
      _oZero->reset(); //zero 플래그와이어초기화

      //입력 와이어에서 값 읽기
      Wire<32> input0_val = *_iInput0;
      Wire<32> input1_val = *_iInput1;
      unsigned long alu_op = _iALUControl->to_ulong(); // 4비트 제어 신호

      Wire<32> result_val;
      switch (alu_op) {
        case 0x0: // 0000: AND
          result_val = input0_val & input1_val;
          break;
        case 0x1: // 0001: OR
          result_val = input0_val | input1_val;
          break;
        case 0x2: // 0010: add
          {
            int32_t val0 = static_cast<int32_t>(input0_val.to_ulong());
            int32_t val1 = static_cast<int32_t>(input1_val.to_ulong());
            result_val = val0 + val1;
          }
          break;
        case 0x6: // 0110: subtract
          {
            int32_t val0 = static_cast<int32_t>(input0_val.to_ulong());
            int32_t val1 = static_cast<int32_t>(input1_val.to_ulong());
            result_val = val0 - val1;
          }
          break;
        case 0x7: // 0111: set on less than (signed)
          {
            int32_t val0_signed = static_cast<int32_t>(input0_val.to_ulong());
            int32_t val1_signed = static_cast<int32_t>(input1_val.to_ulong());
            result_val = (val0_signed < val1_signed) ? 1 : 0;
          }
          break;
        case 0xC: // 1100: NOR
          result_val = ~(input0_val | input1_val);
          break;
        default:
          assert(false && "ALU: Undefined ALU control signal");
          break;
      }

      *_oOutput = result_val;
      _oZero->set(0, result_val.none()); // 결과가 0이면 _oZero를 1로 설정
    }

  private:

    const Wire<4> *_iALUControl;
    const Wire<32> *_iInput0;
    const Wire<32> *_iInput1;
    Wire<32> *_oOutput;
    Wire<1> *_oZero;

};

#endif

