#include <aris.hpp>

#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"
#include "kaanh/middleware/middleware.hpp"


namespace testSpace
{

	class ZeroG : public aris::core::CloneObject<ZeroG, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt() -> void;
		auto virtual executeRT() -> int;

		virtual ~ZeroG();
		explicit ZeroG(const std::string &name = "zerog");
        KAANH_DECLARE_BIG_FOUR(ZeroG)

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
    
}