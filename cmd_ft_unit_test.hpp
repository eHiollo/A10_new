#ifndef CMD_FT_UNIT_TEST_HPP
#define CMD_FT_UNIT_TEST_HPP

#include "aris.hpp"
#include "kaanh/general/macro.hpp"

/**
 * @brief 力传感器最小单元测试命令
 *
 * 功能：
 * 1. 按控制周期连续读取六维力传原始数据
 * 2. 按默认 10s 或用户指定时长持续记录
 * 3. 将原始整型值和缩放后的 double 值写入日志
 *
 * 使用方法：
 * ft_unit --ft_id=8 --time=10
 * ft_unit -f=15 -t=5
 */
class CmdFtUnitTest : public aris::core::CloneObject<CmdFtUnitTest, aris::plan::Plan> {
public:
    auto virtual prepareNrt() -> void override;
    auto virtual executeRT() -> int override;
    auto virtual collectNrt() -> void override;

    virtual ~CmdFtUnitTest();
    explicit CmdFtUnitTest(const std::string& name = "CmdFtUnitTest");
    KAANH_DECLARE_BIG_FOUR(CmdFtUnitTest);

private:
    struct Imp;
    aris::core::ImpPtr<Imp> imp_;
};

#endif  // CMD_FT_UNIT_TEST_HPP
