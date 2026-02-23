#include "Cache.h"
#include <cstring> // For memcpy

// '_memory' defined in testCache.cc
extern BYTE _memory[MEMSIZE];

/*************************************************************************************/
/* Assignment #4: Caches                                                             */
/*                                                                                   */
/* addr: target memory address                                                       */
/* isWrite: true if write, false if read                                             */
/* data: data to be written to addr if isWrite, data read from the cache if !isWrite */
/*************************************************************************************/
void Cache::access(const WORD addr, const bool isWrite, WORD *data) {
  assert(addr % 4 == 0);
  assert(data != nullptr);

  // 1. 주소 분해 및 기본 정보 계산
  size_t numWays = _numLines / _numSets;
  WORD byteOffset = addr & (_lineSize - 1);
  WORD blockAddr = addr / _lineSize;
  WORD setIndex = blockAddr & (_numSets - 1);
  WORD tag = blockAddr / _numSets;

  // 2. LRU 카운터 업데이트
  for (size_t way = 0; way < numWays; way++){
    if (_lines[setIndex][way].valid){
      _lines[setIndex][way].lruCounter++;
    }
  }

  // 3. 캐시 조회 (Cache Lookup) 및 히트(Hit) 처리
  int hitWay = -1; // hit된 웨이의 인덱스, -1은 미스를 의미

  for(size_t way = 0; way < numWays; way++){
    if (_lines[setIndex][way].valid && _lines[setIndex][way].tag == tag){
      // 캐시 히트!
      hitWay = way;
      _lines[setIndex][hitWay].lruCounter = 0; // LRU 카운터 리셋

      if (!isWrite) { // 읽기 히트(Read Hit)
        // 캐시 라인에서 데이터를 읽어 *data에 저장
        memcpy(data, &(_lines[setIndex][hitWay].data[byteOffset]), sizeof(WORD));
      } else { // 쓰기 히트(Write Hit)
        // *data의 값을 캐시 라인에 씀
        memcpy(&(_lines[setIndex][hitWay].data[byteOffset]), data, sizeof(WORD));
        if(_writePolicy == WRITE_THROUGH){
          // WRITE_THROUGH 정책 : 메인 메모리에도 즉시 씀
          memcpy(&(_memory[addr]), data, sizeof(WORD));
        } else { // WRITE_BACK 정책
          // WRITE_BACK 정책 : 해당 라인의 dirty 비트를 true로 설정
          _lines[setIndex][hitWay].dirty = true;
        }
      }
      return;
    }
  }

  // 4. 캐시 미스 처리
  // 캐시 히트가 발생하지 않았을 떄 실행 (hitWay == -1)
  // 4a. 희생 라인(Victim line) 선택
  int victimWay = -1;

  // invalid line을 찾는다.
  for (size_t way = 0; way < numWays; ++way){
    if (!_lines[setIndex][way].valid){
      victimWay = way;
      break; // 가장 작은 인덱스의 유효하지 않는 라인을 선택
    } 
  }

  // 유효하지 않은 라인이 없다면 (모든 라인인 유효하다면), LRU 정책에 따라 희생 라인을 선택
  if (victimWay == -1){
    size_t maxLruCounter = 0;
    victimWay = 0;
    for(size_t way = 0; way < numWays; ++way){
      if (_lines[setIndex][way].lruCounter >= maxLruCounter){
        maxLruCounter = _lines[setIndex][way].lruCounter;
        victimWay = way;
      }
    }
  }

  // 4b. 희생 라인 쓰기 후기 (Write-Back) 처리 - 필요한 경우
  if (_lines[setIndex][victimWay].valid && _writePolicy == WRITE_BACK && _lines[setIndex][victimWay].dirty){
    WORD victimTag = _lines[setIndex][victimWay].tag;
    // blockAddr = tag * numSets + setIndex
    WORD victimBlockNum = (victimTag * _numSets) + setIndex;
    WORD victimMemAddr = victimBlockNum * _lineSize;

    // 희생 라인의 데이터를 메인 메모리에 씀
    memcpy(&(_memory[victimMemAddr]), _lines[setIndex][victimWay].data, _lineSize);
  }

  // 4b. 메인 메모리에서 데이터 가져오기
  // 현재 요청된 addr에 해당하는 메모리 블록의 시작 주소를 계산
  WORD blockStartAddr = (addr / _lineSize) * _lineSize;

  // 메인 메모리에서 캐시 라인으로 데이터 복사
  memcpy(_lines[setIndex][victimWay].data, &(_memory[blockStartAddr]), _lineSize);

  // 4c. 희생 라인 메타데이터 업데이트 (명세 4b단계 - 라인 업데이트, 4c단계)
  _lines[setIndex][victimWay].valid = true;
  _lines[setIndex][victimWay].tag = tag; // 현재 요청의 태그로 업데이트
  _lines[setIndex][victimWay].lruCounter = 0; // 명세 4c단계
  if (_writePolicy == WRITE_BACK) {
    _lines[setIndex][victimWay].dirty = false; // 새로 가져온 데이터는 아직 dirty하지 않음
  }

  // 4. 원래 요청 처리 (이제 데이터는 캐시에 있음)
  if (!isWrite) { // 읽기 미스 (Read Miss)
    memcpy(data, &(_lines[setIndex][victimWay].data[byteOffset]), sizeof(WORD));
  } else { // 쓰기 미스 (Write Miss)
    memcpy(&(_lines[setIndex][victimWay].data[byteOffset]), data, sizeof(WORD));
    if (_writePolicy == WRITE_THROUGH) { // WRITE_THROUGH 정책: 메인 메모리에도 즉시 씀
      memcpy(&(_memory[addr]), data, sizeof(WORD));
    } else { // WRITE_BACK 정책: 해당 라인의 dirty 비트를 true로 설정
      _lines[setIndex][victimWay].dirty = true;
    }
  }
}

