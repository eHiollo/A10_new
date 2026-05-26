#ifndef KAANH_TEMPLATE_HPP_
#define KAANH_TEMPLATE_HPP_

#include "aris.hpp"
#include "kaanh/general/macro.hpp"
#include "kaanh/module/middle_module.hpp"
#include "kaanh/middleware/middleware.hpp"
namespace kaanhtemplate {
//hpp中的plan模板
class KAANH_API PlanName :public aris::core::CloneObject<PlanName,aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void override;
    auto virtual executeRT()->int override;
    auto virtual collectNrt()->void override;
    explicit PlanName (const std::string &name = "PlanName");
    KAANH_DECLARE_BIG_FOUR(PlanName)
    virtual ~PlanName ();
private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_; 
};

//hpp中的module模板
class KAANH_API ModuleName : public kaanh::module::MiddleModule {
    public:
        auto virtual init()->void override;
        auto virtual execute(const std::string &str, std::function<void(std::string)> send_ret) noexcept->std::pair<std::string, std::string> override;    
        auto function1()->void;
        //...func
        static auto instanceInCs()->ModuleName&;
    private:
        struct Imp;
        std::unique_ptr<Imp> imp_;
    public:
        ModuleName();
        ~ModuleName();
        KAANH_DELETE_BIG_FOUR(ModuleName)
};

//hpp中的全局函数
KAANH_API auto function2()->int;

//需要导出的模板：两种方法
//1、显示声明并用KAANH_API导出模板的特化
//2、或者声明和实现均放在头文件



} // namespace kaanhtemplate

#endif //KAANH_TEMPLATE_HPP_