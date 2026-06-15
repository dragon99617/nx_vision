#pragma once

#include "common/config.hpp"

#include <string>

namespace nxv {

class SerialPort {
public:
    ~SerialPort();

    bool open(const SerialConfig &config);
    bool write_line(const std::string &line);
    void close();
    bool is_open() const;

private:
    SerialConfig config_;
    int fd_ = -1;
};

}  // namespace nxv

