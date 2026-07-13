#pragma once

#include "common/config.hpp"
#include "common/types.hpp"

#include <string>

namespace nxv {

class GimbalProtocol {
public:
    explicit GimbalProtocol(SerialConfig config);
    std::string make_packet(const AimResult &aim,
                            const DepthEstimate &depth,
                            const FrameBundle &frame,
                            double generation_timestamp_s = -1.0) const;

private:
    std::string make_legacy_packet(const AimResult &aim) const;
    std::string make_v2_packet(const AimResult &aim,
                               const DepthEstimate &depth,
                               const FrameBundle &frame,
                               double generation_timestamp_s) const;

    SerialConfig config_;
};

}  // namespace nxv
