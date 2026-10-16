#pragma once
#include <Arduino.h>

// ASCII-hex register protocol driver for the distance sensor(s).
// Optimized for AVR (Leonardo/32u4): no sprintf/strtol, minimal buffer
// sizes, shared ack-command path, bounded wait times.
class DistanceSensor
{
  public:
  // timeoutMs bounds each individual stream read (used for readBytesUntil).
  DistanceSensor(Stream* stream, uint8_t deviceIndex, unsigned long timeoutMs = 20)
    : stream(stream), mDeviceIndex(deviceIndex)
  {
    stream->setTimeout(timeoutMs);
  }

  bool readRegister(uint8_t index, uint16_t* result)
  {
    char buffer[8]; // "R" + 2 + 2 + "\n" = 6, +1 spare, no trailing NUL needed
    uint8_t len = encodeCmd(buffer, 'R', index);
    stream->write(buffer, len);

    char resp[5]; // up to 4 hex digits, no NUL needed
    int rlen = stream->readBytesUntil('\n', resp, sizeof(resp));
    if (rlen <= 0 || rlen > 4) return false;

    *result = decodeHex(resp, rlen);
    return true;
  }

  bool writeRegister(uint8_t index, uint16_t value)
  {
    char buffer[10]; // "W" + 2 + 2 + 3 + "\n" = 9, +1 spare
    uint8_t len = encodeCmdVal(buffer, 'W', index, value);
    return sendAndExpectAck(buffer, len);
  }

  bool saveConfiguration()
  {
    return sendSimpleCommand('S');
  }

  bool restartSensor()
  {
    return sendSimpleCommand('U');
  }

  // Kept for backward compatibility with the original (misspelled) API.
  bool rsetartSensor() { return restartSensor(); }

  bool resetConfiguration()
  {
    return sendSimpleCommand('Z');
  }

  unsigned int readDistance_mm()
  {
    uint16_t res = 0;
    return readRegister(0x01, &res) ? res : 0xFFF;
  }

  unsigned int readDistance_cm()
  {
    uint16_t res = 0;
    return readRegister(0x06, &res) ? res : 0xFF;
  }

  // Ranging status (register 0x00): bit 0 = measurement valid, bits 1..7 = failed checks.
  unsigned int readStatus()
  {
    uint16_t res = 0;
    return readRegister(0x00, &res) ? res : 0xFF; // 0xFF also flags "no response / error"
  }

  bool waitForBoot(unsigned long timeoutMs = 500)
  {
    unsigned long deadline = millis() + timeoutMs;
    uint16_t tmp;
    do
    {
      if (readRegister(0x00, &tmp)) return true;
    } while ((long)(deadline - millis()) > 0);
    return false;
  }

  uint8_t mDeviceIndex;

  private:
  Stream* stream;

  static inline char nibble(uint8_t v)
  {
    static const char digits[16] PROGMEM = "0123456789ABCDEF";
    return pgm_read_byte(&digits[v & 0xF]);
  }

  static inline void putByteHex(char* dst, uint8_t v)
  {
    dst[0] = nibble(v >> 4);
    dst[1] = nibble(v);
  }

  // Builds "C%02X%02X\n" -> returns length (6), no NUL byte written.
  uint8_t encodeCmd(char* dst, char cmd, uint8_t index)
  {
    dst[0] = cmd;
    putByteHex(dst + 1, mDeviceIndex);
    putByteHex(dst + 3, index);
    dst[5] = '\n';
    return 6;
  }

  // Builds "C%02X%02X%03X\n" -> returns length (9), no NUL byte written.
  uint8_t encodeCmdVal(char* dst, char cmd, uint8_t index, uint16_t value)
  {
    dst[0] = cmd;
    putByteHex(dst + 1, mDeviceIndex);
    putByteHex(dst + 3, index);
    dst[5] = nibble((value >> 8) & 0xF);
    dst[6] = nibble((value >> 4) & 0xF);
    dst[7] = nibble(value & 0xF);
    dst[8] = '\n';
    return 9;
  }

  static inline uint16_t decodeHex(const char* s, uint8_t len)
  {
    uint16_t v = 0;
    for (uint8_t i = 0; i < len; i++)
    {
      char c = s[i];
      uint8_t d = (c <= '9') ? (c - '0') : ((c & 0x1F) + 9); // handles 'A'-'F' and 'a'-'f'
      v = (uint16_t)(v << 4) | d;
    }
    return v;
  }

  // Sends cmd/len, expects exactly one byte back equal to 'A' (0xA ack).
  bool sendAndExpectAck(const char* buffer, uint8_t len)
  {
    stream->write(buffer, len);
    char resp[2];
    int rlen = stream->readBytesUntil('\n', resp, sizeof(resp));
    return rlen == 1 && resp[0] == 'A';
  }

  // Shared path for S / U / Z commands: "C%02X\n" + single-byte ack.
  bool sendSimpleCommand(char cmd)
  {
    char buffer[5]; // cmd + 2 + "\n" = 4, +1 spare
    buffer[0] = cmd;
    putByteHex(buffer + 1, mDeviceIndex);
    buffer[3] = '\n';
    return sendAndExpectAck(buffer, 4);
  }
};
