#ifndef MoveLL_HPP_
#define MoveLL_HPP_

#include <string>
#include <aris.hpp>

namespace my_cmd{
class MoveLL : public aris::core::CloneObject<MoveLL, aris::plan::Plan>
{
public:
    auto prepareNrt() -> void override;
    auto executeRT() -> int override;
    // 计算六维位置数组之间的距离
    static double Dis(const double arr1[6], const double arr2[6]);

    virtual ~MoveLL();
    explicit MoveLL(const std::string& name = "MoveLL");

private:
	struct Imp;
	aris::core::ImpPtr<Imp> imp_;
};
}
#endif

