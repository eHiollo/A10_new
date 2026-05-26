#pragma once

#include <aris.hpp>
#include <memory>

namespace tactile_collect_real
{
    class TactileCollectReal : public aris::core::CloneObject<TactileCollectReal, aris::plan::Plan>
    {
    public:
        struct Imp;
        std::unique_ptr<Imp> imp_;

        auto virtual prepareNrt() -> void override;
        auto virtual executeRT() -> int override;

        TactileCollectReal(const std::string& name = "tactile_collect_real");
        TactileCollectReal(const TactileCollectReal& other);
        ~TactileCollectReal();
    };
}
