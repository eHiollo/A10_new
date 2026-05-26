#include <aris.hpp>

#include "curve.hpp"
#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"
#include "kaanh/middleware/middleware.hpp"


const double PI = aris::PI;

namespace robot
{


	class ModelInit : public aris::core::CloneObject<ModelInit, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		virtual ~ModelInit();
		explicit ModelInit(const std::string& name = "init");
		KAANH_DECLARE_BIG_FOUR(ModelInit)

	private:
		struct Imp;
        aris::core::ImpPtr<Imp> imp_;

	};



	class ModelForward :public aris::core::CloneObject<ModelForward, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelForward();
		explicit ModelForward(const std::string& name = "ModelForward");
		KAANH_DECLARE_BIG_FOUR(ModelForward)

	private:

	};



	class ModelGet : public aris::core::CloneObject<ModelGet, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelGet();
		explicit ModelGet(const std::string& name = "ModelGet");
		KAANH_DECLARE_BIG_FOUR(ModelGet)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

	};



    class ModelTest :public aris::core::CloneObject<ModelTest, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

        virtual ~ModelTest();
        explicit ModelTest(const std::string& name = "ModelTest");
        KAANH_DECLARE_BIG_FOUR(ModelTest)
	private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
	};


    class ModelSetPos :public aris::core::CloneObject<ModelSetPos, aris::plan::Plan>
        {
        public:
            auto virtual prepareNrt()->void;
            auto virtual executeRT()->int;

            virtual ~ModelSetPos();
            explicit ModelSetPos(const std::string& name = "model_set_pos");
            KAANH_DECLARE_BIG_FOUR(ModelSetPos)
         private:
			 struct Imp;
			 aris::core::ImpPtr<Imp> imp_;

        };

	class ModelComP :public aris::core::CloneObject<ModelComP, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ModelComP();
		explicit ModelComP(const std::string& name = "ModelComP");
		KAANH_DECLARE_BIG_FOUR(ModelComP)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class ForceAlign :public aris::core::CloneObject<ForceAlign, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceAlign();
		explicit ForceAlign(const std::string& name = "ForceAlign");
		KAANH_DECLARE_BIG_FOUR(ForceAlign)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};

	class ForceKeep :public aris::core::CloneObject<ForceKeep, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceKeep();
		explicit ForceKeep(const std::string& name = "ForceKeep");
		KAANH_DECLARE_BIG_FOUR(ForceKeep)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;


	};

	class ForceDrag :public aris::core::CloneObject<ForceDrag, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~ForceDrag();
		explicit ForceDrag(const std::string& name = "ForceDrag");
		KAANH_DECLARE_BIG_FOUR(ForceDrag)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;



	};








    class Demo :public aris::core::CloneObject<Demo, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~Demo();
        explicit Demo(const std::string& name = "Demo");
        KAANH_DECLARE_BIG_FOUR(Demo)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };





	class PegOutHole :public aris::core::CloneObject<PegOutHole, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PegOutHole();
		explicit PegOutHole(const std::string& name = "PegOutHole");
		KAANH_DECLARE_BIG_FOUR(PegOutHole)
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

	};






    class Arm2PegInHole :public aris::core::CloneObject<Arm2PegInHole, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~Arm2PegInHole();
        explicit Arm2PegInHole(const std::string& name = "Arm2PegInHole");
        KAANH_DECLARE_BIG_FOUR(Arm2PegInHole)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

    class PlateAllignData :public aris::core::CloneObject<PlateAllignData, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~PlateAllignData();
        explicit PlateAllignData(const std::string& name = "PlateAllignData");
        KAANH_DECLARE_BIG_FOUR(PlateAllignData)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };

    class PlateAllignTest :public aris::core::CloneObject<PlateAllignTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~PlateAllignTest();
        explicit PlateAllignTest(const std::string& name = "PlateAllignTest");
        KAANH_DECLARE_BIG_FOUR(PlateAllignTest)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;


    };



    class Search : public aris::core::CloneObject<Search, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        virtual ~Search();
        explicit Search(const std::string& name = "Search");
        KAANH_DECLARE_BIG_FOUR(Search)


    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;

    };


	class PressDown :public aris::core::CloneObject<PressDown, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~PressDown();
		explicit PressDown(const std::string& name = "PressDown");
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

    class JointTrajCli : public aris::core::CloneObject<JointTrajCli, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt() -> void;
        auto virtual executeRT() -> int;

        virtual ~JointTrajCli();
        explicit JointTrajCli(const std::string &name = "JointTrajCli");
        KAANH_DECLARE_BIG_FOUR(JointTrajCli)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };

    class PoseTrajCli : public aris::core::CloneObject<PoseTrajCli, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt() -> void;
        auto virtual executeRT() -> int;

        virtual ~PoseTrajCli();
        explicit PoseTrajCli(const std::string &name = "PoseTrajCli");
        KAANH_DECLARE_BIG_FOUR(PoseTrajCli)

    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };





}




