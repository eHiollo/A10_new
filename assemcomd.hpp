#ifndef ASSEMCOMD_H
#define ASSEMCOMD_H

#include <aris.hpp>

#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"
#include "kaanh/middleware/middleware.hpp"


namespace assemble
{

    class HoleInPeg :public aris::core::CloneObject<HoleInPeg, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~HoleInPeg();
        explicit HoleInPeg(const std::string& name = "HoleInPeg");
        KAANH_DECLARE_BIG_FOUR(HoleInPeg)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

    class PegInHole :public aris::core::CloneObject<PegInHole, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~PegInHole();
        explicit PegInHole(const std::string& name = "PegInHole");
        KAANH_DECLARE_BIG_FOUR(PegInHole)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

    class Insert :public aris::core::CloneObject<Insert, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~Insert();
        explicit Insert(const std::string& name = "Insert");
        KAANH_DECLARE_BIG_FOUR(Insert)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };



}






#endif // ASSEMCOMD_H
