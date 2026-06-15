#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

#include <string>

namespace nxv {

class GimbalProtocol {
public:
    explicit GimbalProtocol(SerialConfig config);
    std::string make_packet(const AimResult &aim) const;
    std::string make_invalid_packet() const;

private:
    SerialConfig config_;
};

}  // namespace nxv

