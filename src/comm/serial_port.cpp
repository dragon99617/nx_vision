//串口通信实现
#include "comm/serial_port.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace nxv {
namespace {

#ifndef _WIN32
std::string resolve_serial_device(const std::string &configured_device)
{
    if (configured_device != "auto") {
        return configured_device;
    }

    std::vector<std::string> candidates;
    const std::filesystem::path by_id_dir("/dev/serial/by-id");
    if (std::filesystem::exists(by_id_dir)) {
        for (const auto &entry : std::filesystem::directory_iterator(by_id_dir)) {
            candidates.push_back(entry.path().string());
        }
    }
    for (const char *prefix : {"/dev/ttyACM", "/dev/ttyUSB"}) {
        for (int i = 0; i < 10; ++i) {
            const std::string path = std::string(prefix) + std::to_string(i);
            if (std::filesystem::exists(path)) {
                candidates.push_back(path);
            }
        }
    }

    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    return candidates.empty() ? configured_device : candidates.front();
}

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
    const std::string device = resolve_serial_device(config.device);
    if (device == "auto") {
        std::cerr << "[serial] no USB serial device found. Expected /dev/ttyACM*, /dev/ttyUSB*, or /dev/serial/by-id/*\n";
        return false;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        perror("[serial] open");
        return false;
    }
    config_.device = device;

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

    std::cout << "[serial] opened " << device << " @ " << config.baudrate << "\n";
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
    size_t offset = 0;
    while (offset < line.size()) {
        const ssize_t written = ::write(fd_, line.data() + offset, line.size() - offset);
        if (written > 0) {
            offset += static_cast<size_t>(written);
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd {};
            pfd.fd = fd_;
            pfd.events = POLLOUT;
            const int ready = ::poll(&pfd, 1, 10);
            if (ready > 0) {
                continue;
            }
            return false;
        }
        std::cerr << "[serial] write failed: " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
#endif
}

int SerialPort::read_available(void *data, std::size_t capacity)
{
    if (data == nullptr || capacity == 0 || !config_.enabled || config_.dry_run) {
        return 0;
    }
#ifdef _WIN32
    return -1;
#else
    if (fd_ < 0) {
        return -1;
    }
    const ssize_t count = ::read(fd_, data, capacity);
    if (count >= 0) {
        return static_cast<int>(count);
    }
    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
    }
    return -1;
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
