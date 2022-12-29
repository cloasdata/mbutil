#ifndef mbutil_h
#define  mbutil_h
#include <Arduino.h>

namespace modbusUtil
{
  struct parserState
  {
    uint16_t errorCode;
    uint16_t noOfError = 0;
    uint16_t state;
    uint16_t crc;
    uint16_t registerRequested;
    uint16_t bytesToReceive;
    uint16_t dataEnd;
    uint8_t startOffset;
    uint8_t crcOffset;
    uint16_t frameLength;
    uint16_t recentFrameIndex;
    uint16_t recentSymbol;
    uint16_t* requestFrame;
    //#ifdef debug
      int8_t _responseFrame[100]; // in case of debug
    //#endif
    uint16_t lenResponseFrame;
    uint32_t timestamp = 0;
  };

  uint16_t ModRTU_CRC(uint16_t buf[], int len);
  uint16_t ModRTU_CRC(byte buf[], int len);

  bool send(Stream *stream, uint16_t requestFrame[]);

  uint8_t* renderRequest(uint8_t slaveId, uint8_t functionCode, uint16_t noOfRegister, uint16_t startAdress);
  
  int16_t retrieve(Stream* stream, uint16_t requestFrame[], float dataOut[], uint16_t bufSize = 100);
  int16_t retrieve(Stream *stream, uint16_t requestFrame[], float dataOut[], uint16_t bufSize, parserState* state);
  
  int16_t parse(uint16_t data[], uint16_t lenData, uint16_t requestFrame[], float dataOut[], parserState* state);
  int16_t parse(uint16_t data[], uint16_t lenData ,uint16_t requestFrame[], float dataOut[]);

  uint16_t calcFrameSize(uint16_t noOfRegister, uint16_t registerSize);
}
#endif