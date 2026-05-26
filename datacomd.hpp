#ifndef ASSEMCOMD_H
#define ASSEMCOMD_H

#include <aris.hpp>

#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"
#include "kaanh/middleware/middleware.hpp"


namespace data
{

    class HoleMove :public aris::core::CloneObject<HoleMove, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~HoleMove();
        explicit HoleMove(const std::string& name = "HoleMove");
        KAANH_DECLARE_BIG_FOUR(HoleMove)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };


    class GetData :public aris::core::CloneObject<GetData, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~GetData();
        explicit GetData(const std::string& name = "GetData");
        KAANH_DECLARE_BIG_FOUR(GetData)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

    class GetHdData :public aris::core::CloneObject<GetHdData, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~GetHdData();
        explicit GetHdData(const std::string& name = "GetHdData");
        KAANH_DECLARE_BIG_FOUR(GetHdData)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };



}




#endif
