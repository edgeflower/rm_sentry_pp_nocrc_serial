#pragma once
#include <cstdint>
#include <string>

class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort();

  bool open(const std::string& dev, int baudrate);
  void close();
  bool isOpen() const;

  // timeout_ms：0=不等待；>0 用 select 等待
  int readSome(uint8_t* out, int len, int timeout_ms);
  bool writeAll(const uint8_t* data, int len);

  // 可选：拉起 DTR/RTS（某些 STM32 VCP 需要）
  void setDtrRts(bool enable);

private:
  int fd_{-1};
  
};
