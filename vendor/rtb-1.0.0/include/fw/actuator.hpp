#ifndef RTB_FRAMEWORK_ACTUATOR_HPP
#define RTB_FRAMEWORK_ACTUATOR_HPP

#include "fw/object.hpp"

namespace rtb::fw {

class ActuatorBase : public ObjectRoot {
public:
    using ObjectRoot::ObjectRoot;

    virtual auto reset() -> void {}

    virtual auto stop() -> void {}
};

}  // namespace rtb::fw

#endif  // RTB_FRAMEWORK_ACTUATOR_HPP