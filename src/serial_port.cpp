#include "rm_sentry_pp_nocrc_serial/serial_port.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>

SerialPort::~SerialPort() { close(); }

static speed_t toBaud(int b) {
  switch (b) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}

bool SerialPort::open(const std::string& dev, int baudrate) {
  close();

  fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  cfmakeraw(&tty);

  speed_t br = toBaud(baudrate);
  cfsetispeed(&tty, br);
  cfsetospeed(&tty, br);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const { return fd_ >= 0; }

int SerialPort::readSome(uint8_t* out, int len, int timeout_ms) {
  if (fd_ < 0) return -1;

  if (timeout_ms > 0) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);

    timeval tv{};
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int rc = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (rc <= 0) return 0;
  }

  int n = ::read(fd_, out, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }
  return n;
}

bool SerialPort::writeAll(const uint8_t* data, int len) {
  if (fd_ < 0) return false;

  int written = 0;
  while (written < len) {
    int n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        usleep(1000);
        continue;
      }
      return false;
    }
    written += n;
  }
  return true;
}

void SerialPort::setDtrRts(bool enable) {
  if (fd_ < 0) return;
  int flags = 0;
  if (ioctl(fd_, TIOCMGET, &flags) != 0) return;
  if (enable) flags |= (TIOCM_DTR | TIOCM_RTS);
  else        flags &= ~(TIOCM_DTR | TIOCM_RTS);
  ioctl(fd_, TIOCMSET, &flags);
}
