#ifndef __MEMORY_H__
#define __MEMORY_H__

#include "DigitalCircuit.h"

#include <cassert>
#include <cstdio>

#define MEMORY_SIZE (32 * 1024 * 1024) // <-- 32-MB memory

class Memory : public DigitalCircuit {

  public:

    enum Endianness { LittleEndian, BigEndian };

    Memory(const std::string &name,
           const Wire<32> *iAddress,
           const Wire<32> *iWriteData,
           const Wire<1> *iMemRead,
           const Wire<1> *iMemWrite,
           Wire<32> *oReadData,
           const Endianness endianness,
           const char *initFileName = nullptr) : DigitalCircuit(name) {
      _endianness = endianness;
      _iAddress = iAddress;
      _iWriteData = iWriteData;
      _oReadData = oReadData;
      _iMemRead = iMemRead;
      _iMemWrite = iMemWrite;

      _memory = new std::bitset<8>[MEMORY_SIZE];

      if (initFileName != nullptr) {
        // Each line of the memory initialization file consists of:
        //   - the starting memory address of a 32-bit data in hexadecimal value
        //   - the eight-digit hexadecimal value of the data
        // e.g., "1000 ABCD1234" stores 0xABCD1234 from 0x1000 to 0x1003.
        FILE *initFile = fopen(initFileName, "r");
        assert(initFile != NULL);
        std::uint32_t addr, value;
        while (fscanf(initFile, " %x %x", &addr, &value) == 2) {
          printf("INFO: memory[0x%08lx..0x%08lx] <-- 0x%08lx\n",
                 (unsigned long)addr, (unsigned long)(addr + 3),
                 (unsigned long)value);
          if (_endianness == LittleEndian) {
            _memory[addr + 0] = (std::uint8_t)(value % 0x100); value >>= 8;
            _memory[addr + 1] = (std::uint8_t)(value % 0x100); value >>= 8;
            _memory[addr + 2] = (std::uint8_t)(value % 0x100); value >>= 8;
            _memory[addr + 3] = (std::uint8_t)(value % 0x100);
          } else { // _endianness == BigEndian
            _memory[addr + 3] = (std::uint8_t)(value % 0x100); value >>= 8;
            _memory[addr + 2] = (std::uint8_t)(value % 0x100); value >>= 8;
            _memory[addr + 1] = (std::uint8_t)(value % 0x100); value >>= 8;
            _memory[addr + 0] = (std::uint8_t)(value % 0x100);
          }
        }
        fclose(initFile);
      }
    }

    void printMemory() {
      for (size_t i = 0; i < MEMORY_SIZE; i += 4) {
        if (_memory[i].any() || _memory[i + 1].any() || _memory[i + 2].any()
            || _memory[i + 3].any()) {
          std::uint32_t value = 0;
          value += (_memory[i + 3].to_ulong()); value <<= 8;
          value += (_memory[i + 2].to_ulong()); value <<= 8;
          value += (_memory[i + 1].to_ulong()); value <<= 8;
          value += (_memory[i + 0].to_ulong());
          printf("  memory[0x%08lx..0x%08lx] = 0x%08lx\n", i, i+3,
                 (unsigned long)value);
        }
      }
    }

    virtual void advanceCycle() {
      unsigned long addr_val = _iAddress->to_ulong();
      // 메모리 읽기 동작 (iMemRead가 1일 때)
       if (_iMemRead->test(0)) {
        std::uint32_t read_value = 0;
        if (_endianness == LittleEndian) {
          // Little Endian: address+0에 LSB, address+3에 MSB
          read_value |= (_memory[addr_val + 0].to_ulong() << 0);
          read_value |= (_memory[addr_val + 1].to_ulong() << 8);
          read_value |= (_memory[addr_val + 2].to_ulong() << 16);
          read_value |= (_memory[addr_val + 3].to_ulong() << 24);
        } else { // BigEndian
          // Big Endian: address+0에 MSB, address+3에 LSB
          read_value |= (_memory[addr_val + 0].to_ulong() << 24);
          read_value |= (_memory[addr_val + 1].to_ulong() << 16);
          read_value |= (_memory[addr_val + 2].to_ulong() << 8);
          read_value |= (_memory[addr_val + 3].to_ulong() << 0);
        }
        *_oReadData = read_value;
      }

      // 메모리 쓰기 동작 (iMemWrite가 1일 때)
      if (_iMemWrite->test(0)) {
        std::uint32_t data_to_write = _iWriteData->to_ulong();
        if (_endianness == LittleEndian) {
          // Little Endian: address+0에 LSB, address+3에 MSB
          _memory[addr_val + 0] = (std::uint8_t)((data_to_write >> 0) & 0xFF);
          _memory[addr_val + 1] = (std::uint8_t)((data_to_write >> 8) & 0xFF);
          _memory[addr_val + 2] = (std::uint8_t)((data_to_write >> 16) & 0xFF);
          _memory[addr_val + 3] = (std::uint8_t)((data_to_write >> 24) & 0xFF);
        } else { // BigEndian
          // Big Endian: address+0에 MSB, address+3에 LSB
          _memory[addr_val + 0] = (std::uint8_t)((data_to_write >> 24) & 0xFF);
          _memory[addr_val + 1] = (std::uint8_t)((data_to_write >> 16) & 0xFF);
          _memory[addr_val + 2] = (std::uint8_t)((data_to_write >> 8) & 0xFF);
          _memory[addr_val + 3] = (std::uint8_t)((data_to_write >> 0) & 0xFF);
        }
      }
    }

    ~Memory() {
      delete[] _memory;
    }

  private:

    const Wire<32> *_iAddress;
    const Wire<32> *_iWriteData;
    const Wire<1> *_iMemRead;
    const Wire<1> *_iMemWrite;
    Wire<32> *_oReadData;

    Endianness _endianness;
    std::bitset<8> *_memory;

};

#endif

