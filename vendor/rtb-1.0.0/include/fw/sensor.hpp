#ifndef RTB_FRAMEWORK_SENSOR_HPP
#define RTB_FRAMEWORK_SENSOR_HPP

#include "fw/object.hpp"

namespace rtb::fw {

class SensorBase : public ObjectRoot {
public:
    using ObjectRoot::ObjectRoot;

    auto init() -> void override {}

    virtual auto update() -> void {};

    virtual auto getData(double* data) -> bool = 0;

    virtual auto isRunning() const -> bool { return true; }
};

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_SENSOR_HPP
