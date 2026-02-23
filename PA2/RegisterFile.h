#ifndef __REGISTER_FILE_HPP__
#define __REGISTER_FILE_HPP__

#include "DigitalCircuit.h"

#include <cassert>
#include <cstdio>

class RegisterFile : public DigitalCircuit {

  public:

    RegisterFile(const Wire<5> *iReadRegister1,
                 const Wire<5> *iReadRegister2,
                 const Wire<5> *iWriteRegister,
                 const Wire<32> *iWriteData,
                 const Wire<1> *iRegWrite,
                 Wire<32> *oReadData1,
                 Wire<32> *oReadData2,
                 const char *initFileName = nullptr)
      : DigitalCircuit("RegisterFile") {
      _iReadRegister1 = iReadRegister1;
      _iReadRegister2 = iReadRegister2;
      _iWriteRegister = iWriteRegister;
      _iWriteData = iWriteData;
      _iRegWrite = iRegWrite;
      _oReadData1 = oReadData1;
      _oReadData2 = oReadData2;

      for (size_t i = 0; i < 32; i++) {
        _registers[i].reset();
      }

      if (initFileName != nullptr) {
        // Each line of the memory initialization file consists of:
        //   1) the target register index in decimal value
        //   2) the eight-digit hexadecimal value of the data
        // For example, "10 ABCD1234" stores 0xABCD1234 to register #10.
        FILE *initFile = fopen(initFileName, "r");
        assert(initFile != NULL);
        std::uint32_t reg, val;
        while (fscanf(initFile, " %u %x", &reg, &val) == 2) {
          if (reg != 0) {
            printf("INFO: $%02u <-- 0x%08lx\n", reg, (unsigned long)val);
            _registers[reg] = val;
          }
        }
      }
    }

    void printRegisters() {
      for (size_t i = 0; i < 32; i++) {
        if (_registers[i].any()) {
          printf("  $%02lu = 0x%08lx\n", i, _registers[i].to_ulong());
        }
      }
    }

    virtual void advanceCycle() {
      // --- 읽기 동작 (항상 수행) ---
      // iReadRegister1 와이어에서 첫 번째 읽을 레지스터 번호를 가져옵니다.
      unsigned long read_reg1_idx = _iReadRegister1->to_ulong();
      // iReadRegister2 와이어에서 두 번째 읽을 레지스터 번호를 가져옵니다.
      unsigned long read_reg2_idx = _iReadRegister2->to_ulong();

      // 첫 번째 레지스터 읽기
      if (read_reg1_idx == 0) {
        *_oReadData1 = 0; // 레지스터 $0은 항상 0을 반환합니다.
      } else {
        *_oReadData1 = _registers[read_reg1_idx];
      }

      // 두 번째 레지스터 읽기
      if (read_reg2_idx == 0) {
        *_oReadData2 = 0; // 레지스터 $0은 항상 0을 반환합니다.
      } else {
        *_oReadData2 = _registers[read_reg2_idx];
      }

      // --- 쓰기 동작 (iRegWrite가 1일 때만 수행) ---
      if (_iRegWrite->test(0)) { // iRegWrite의 0번째 비트가 1인지 확인합니다.
        unsigned long write_reg_idx = _iWriteRegister->to_ulong();
        if (write_reg_idx != 0) { // 레지스터 $0에는 쓰기를 무시합니다.
          _registers[write_reg_idx] = *_iWriteData;
        }
      }
    }

  private:

    const Wire<5> *_iReadRegister1;
    const Wire<5> *_iReadRegister2;
    const Wire<5> *_iWriteRegister;
    const Wire<32> *_iWriteData;
    const Wire<1> *_iRegWrite;
    Wire<32> *_oReadData1;
    Wire<32> *_oReadData2;

    std::bitset<32> _registers[32];

};

#endif

