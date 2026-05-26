#include <aris.hpp>

#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"
#include "kaanh/middleware/middleware.hpp"


namespace rivet
{

    class RivetInit : public aris::core::CloneObject<RivetInit, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        virtual ~RivetInit();
        explicit RivetInit(const std::string& name = "RivetInit");
        KAANH_DECLARE_BIG_FOUR(RivetInit)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

    };

    class RivetStart : public aris::core::CloneObject<RivetStart, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        virtual ~RivetStart();
        explicit RivetStart(const std::string& name = "RivetStart");
        KAANH_DECLARE_BIG_FOUR(RivetStart)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

    };

    class RivetOut :public aris::core::CloneObject<RivetOut, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~RivetOut();
		explicit RivetOut(const std::string& name = "RivetOut");
		KAANH_DECLARE_BIG_FOUR(RivetOut)
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

    class RivetSearch :public aris::core::CloneObject<RivetSearch, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~RivetSearch();
        explicit RivetSearch(const std::string& name = "RivetSearch");
        KAANH_DECLARE_BIG_FOUR(RivetSearch)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

    class Arm2Init : public aris::core::CloneObject<Arm2Init, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        virtual ~Arm2Init();
        explicit Arm2Init(const std::string& name = "Arm2Init");
        KAANH_DECLARE_BIG_FOUR(Arm2Init)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

    };

    class RivetHoleDection :public aris::core::CloneObject<RivetHoleDection, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~RivetHoleDection();
        explicit RivetHoleDection(const std::string& name = "RivetHoleDection");
        KAANH_DECLARE_BIG_FOUR(RivetHoleDection)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

    class RivetCalib :public aris::core::CloneObject<RivetCalib, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~RivetCalib();
        explicit RivetCalib(const std::string& name = "RivetCalib");
        KAANH_DECLARE_BIG_FOUR(RivetCalib)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };




}