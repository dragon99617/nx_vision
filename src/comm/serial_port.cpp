//串口通信实现
#include "comm/serial_port.hpp"

#include <iostream>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace nxv {
namespace {

#ifndef _WIN32
speed_t baud_to_speed(int baudrate)
{
    switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default: return B115200;
    }
}
#endif

}  // namespace

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const SerialConfig &config)
{
    config_ = config;
    if (!config.enabled || config.dry_run) {
        std::cout << "[serial] dry-run mode on " << config.device << "\n";
        return true;
    }

#ifdef _WIN32
    std::cerr << "[serial] real serial is not implemented on Windows in this project\n";
    return false;
#else
    fd_ = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        perror("[serial] open");
        return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        perror("[serial] tcgetattr");
        close();
        return false;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, baud_to_speed(config.baudrate));
    cfsetospeed(&tty, baud_to_speed(config.baudrate));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        perror("[serial] tcsetattr");
        close();
        return false;
    }

    return true;
#endif
}

bool SerialPort::write_line(const std::string &line)
{
    if (!config_.enabled || config_.dry_run) {
        (void)line;
        return true;
    }

#ifdef _WIN32
    return false;
#else
    if (fd_ < 0) {
        return false;
    }
    const ssize_t written = ::write(fd_, line.data(), line.size());
    return written == static_cast<ssize_t>(line.size());
#endif
}

void SerialPort::close()
{
#ifndef _WIN32
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#endif
}

bool SerialPort::is_open() const
{
    return config_.dry_run || fd_ >= 0;
}

}  // namespace nxv
