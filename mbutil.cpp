#include <Arduino.h>
#include "Stream.h"
#include "mbutil.h"

namespace modbusUtil
{ 

  enum ModbusErrorCode
  {
    noError = 0,
    illegalFunction = -1,
    CRCError = -21,
    illegalDataValue = -3,
    frameInvalid = -99,

  };

  enum ModbusComm
  {
    error,
    slaveAdress,
    functionCode,
    byteCount,
    dataReceive,
    crcBytes,
  };

  union FloatAssemble
  {
    long toLongInt;
    float toFloat;
  };

  uint16_t ModRTU_CRC(uint16_t buf[], int len){
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++)
    {
      crc ^= buf[pos]; // XOR byte into least sig. byte of crc

      for (int i = 8; i != 0; i--)
      { // Loop over each bit
        if ((crc & 0x0001) != 0)
        {            // If the LSB is set
          crc >>= 1; // Shift right and XOR 0xA001
          crc ^= 0xA001;
        }
        else         // Else LSB is not set
          crc >>= 1; // Just shift right
      }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
  }

  // Compute the MODBUS RTU CRC
  uint16_t ModRTU_CRC(byte buf[], int len)
  {
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++)
    {
      crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc

      for (int i = 8; i != 0; i--)
      { // Loop over each bit
        if ((crc & 0x0001) != 0)
        {            // If the LSB is set
          crc >>= 1; // Shift right and XOR 0xA001
          crc ^= 0xA001;
        }
        else         // Else LSB is not set
          crc >>= 1; // Just shift right
      }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
  }

  bool send(Stream *stream, uint16_t *modbusRequestFrame)
  {
    // send data char by char
    for (int i = 0; i < 8; i++)
    {
      stream->write(*modbusRequestFrame);
      modbusRequestFrame++;
    }
    return true;
  }




  /*
  Renders request from passed data to a 8 byte long array.
  */
  // uint8_t *renderRequest(uint8_t requestFrame[8], uint8_t slaveId, uint8_t functionCode, uint16_t noOfRegister, uint16_t startAdress)
  // {
  //   requestFrame = {slaveId, functionCode, highByte(noOfRegister), lowByte(noOfRegister), highByte(startAdress), lowByte(startAdress)};
  //   uint16_t crc = ModRTU_CRC(requestFrame, 6);
  //   requestFrame[6] = highByte(crc);
  //   requestFrame[7] = lowByte(crc);

  //   return requestFrame;
  // }



  /*
  Reads the stream until no data is available or if buffer is overun.
  User takes care that enough data is already available.
  After data is read, data is parsed.

  */
  uint16_t _retrieve(Stream* stream, uint16_t* dataBuffer, uint16_t bufferSize){
    uint8_t i = 0;
    while (stream->available() && i < bufferSize)
    {
      dataBuffer[i] = stream->read();
      i++;
    }
    return i;
  }


  int16_t retrieve(Stream *stream, uint16_t requestFrame[], float dataOut[], uint16_t bufSize)
  {
    uint16_t dataBuf[bufSize];
    uint16_t bytesRead = _retrieve(stream, dataBuf, bufSize);
    return parse(dataBuf, bytesRead, requestFrame, dataOut);
  }


  int16_t retrieve(Stream *stream, uint16_t requestFrame[], float dataOut[], uint16_t bufSize, parserState* state)
  {
    uint16_t dataBuf[bufSize];
    uint16_t bytesRead = _retrieve(stream, dataBuf, bufSize);
    return parse(dataBuf, bytesRead, requestFrame, dataOut, state);
  }


  int16_t parse(uint16_t data[], uint16_t lenData, uint16_t requestFrame[], float dataOut[], parserState* state)
  {
    
    state->recentFrameIndex = 0;
    state->requestFrame = requestFrame;
    // state->errorCode = noError; keep last error if provided
    state->state = slaveAdress;
    state->registerRequested = requestFrame[5];
    state->bytesToReceive = state->registerRequested * 2;
    state->startOffset = 3;
    state->crcOffset = 2;
    state->frameLength = state->startOffset + state->bytesToReceive + state->crcOffset;
    state->dataEnd = state->startOffset + state->bytesToReceive;
    state->timestamp = millis();
    uint8_t responseFrame[state->frameLength];

    if (lenData < state->frameLength ){
      state->state = error;
      state->errorCode = frameInvalid;
    } 

    while (1)
    {
      responseFrame[state->recentFrameIndex] = data[state->recentFrameIndex];
      state->recentSymbol = data[state->recentFrameIndex];
      //#ifdef debug
        state->_responseFrame[state->recentFrameIndex] = state->recentSymbol;
      //#endif
      switch (state->state)
      {

      case slaveAdress:
        if (state->recentSymbol == requestFrame[0])
        {
          state->state = functionCode;
          state->recentFrameIndex++;
          break;
        }
        else
        {
          // we did not find the slave adress and keep iterating
          if (state->recentFrameIndex < state->frameLength)
          {
            state->recentFrameIndex++;
            break;
          }
          else
          {
            state->errorCode = illegalFunction;
            state->state = error;
            break;
          }
        }

      case functionCode:
        if (state->recentSymbol == requestFrame[1])
        {
          state->state = byteCount;
          state->recentFrameIndex++;
          break;
        }
        else
        {
          state->state = error;
          state->errorCode = illegalFunction;
          break;
        }

      case byteCount:
        if (state->recentSymbol == requestFrame[5] * 2)
        // the response is the actual byteCount (not register count. Here we have 4 bytes per register)
        {
          state->state = dataReceive;
          state->recentFrameIndex++;
          break;
        }
        else
        {
          state->state = error;
          state->errorCode = illegalDataValue;
          break;
        }

      case dataReceive:
      {
        if (state->recentFrameIndex <= state->dataEnd)
        {
          state->recentFrameIndex++;
          break;
        }
        else
        {
          state->recentFrameIndex++;
          state->state = crcBytes;
          break;
        }
      }

      case crcBytes:
      {
        if (state->recentFrameIndex < state->frameLength)
        {
          state->recentFrameIndex++;
          break;
        }
        else
        {

          state->crc = ModRTU_CRC(responseFrame, state->bytesToReceive+state->startOffset);
          if ((responseFrame[state->recentFrameIndex - 1] != highByte(state->crc)) || (responseFrame[state->recentFrameIndex - 2] != lowByte(state->crc)))
          {
            state->state = error;
            state->errorCode = CRCError;
            break;
          }
          else
          { // CRC is fine
            // according docs all values here are 32bit long ieee floats transmitted in 2 16 bit register or 4 bytes
            // as we do need to populate an array of floats,
            // we first assemble the float and than asign it to the array.
            int frameIndex = state->startOffset;
            uint16_t noOfFloats = state->registerRequested / 2;
            for (int c = 0; c < noOfFloats; c++)
            {
              union FloatAssemble assembleFloat;
              assembleFloat.toLongInt = 0x0;
              uint16_t readUntil = frameIndex + 4; // read four bytes
              for (; frameIndex < readUntil; frameIndex++)
              {
                uint8_t dataByte = responseFrame[frameIndex];
                assembleFloat.toLongInt = (assembleFloat.toLongInt << 8) | dataByte;
              }
              dataOut[c] = assembleFloat.toFloat;
            }
            return true;
            break;
          }
        }
      }

      case error:
      {
        state->noOfError++;
        return state->errorCode;
      }

      default:
      {
        state->state = slaveAdress;
        break;
      }
      }
    }
  };

  int16_t parse(uint16_t data[], uint16_t lenData, uint16_t requestFrame[], float dataOut[])
  {
    parserState state = parserState();
    return parse(data, lenData, requestFrame, dataOut, &state);
  };

  uint16_t calcFrameSize(uint16_t noOfRegister, uint16_t registerSize){
    return 2 + noOfRegister * registerSize + 2;
  };

};
