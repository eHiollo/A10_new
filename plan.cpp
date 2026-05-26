#include "plan.hpp"

#include "a10_policy_tcp_plan.hpp"
#include "a10_teleop_tcp_plan.hpp"
#include "cmd_ft_unit_test.hpp"
#include "kaanhbot/utility/kaanh.hpp"
#include "kaanhbot/io/io_plan.hpp"
#include "kaanhbot/functional/function_gather.hpp"
#include "kaanhbot/protocol/modbus_tcp_server.hpp"
#include "kaanhbot/config/external_axis.hpp"
#include "kaanhbot/calibration/calibration.hpp"
#include "kaanh/current/current.hpp"
#include "kaanh/multi_motion/jog_plan.hpp"
#include "kaanh/multi_motion/motion_plan.hpp"
#include "kaanhbot/motion/move_find.hpp"
#include "kaanhbot/conveyor/conveyor_motion.hpp"
#include "kaanh/module/splc_controller.hpp"
#include "robot.hpp"

auto creatPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<kaanhbot::general::Mode>();
    plan_root->planPool().add<kaanhbot::general::Start>();
    plan_root->planPool().add<kaanhbot::general::Stop>();
    plan_root->planPool().add<kaanhbot::general::Clear>();
    plan_root->planPool().add<kaanhbot::general::CancelEStop>();
    plan_root->planPool().add<kaanhbot::general::SaveHome>();
    plan_root->planPool().add<kaanhbot::general::Sleep>();
    plan_root->planPool().add<kaanhbot::general::Recover>();
    plan_root->planPool().add<kaanhbot::general::ManualStop>();
    plan_root->planPool().add<kaanhbot::general::DeleteFile>();
    plan_root->planPool().add<kaanhbot::general::SetDH>();
    plan_root->planPool().add<kaanhbot::general::SaveXml>();
    plan_root->planPool().add<kaanhbot::general::ScanSlave>();
    plan_root->planPool().add<kaanhbot::general::GetXml>();
    plan_root->planPool().add<kaanhbot::general::SetXml>();
    plan_root->planPool().add<kaanhbot::general::SetPgmVel>();
    plan_root->planPool().add<kaanhbot::general::SetJogVel>();
    plan_root->planPool().add<kaanhbot::general::SetTool>();
    plan_root->planPool().add<kaanhbot::general::SetWobj>();
    plan_root->planPool().add<kaanhbot::general::SetJogCoordinate>();
    plan_root->planPool().add<kaanhbot::general::SetJogRunMode>();
    plan_root->planPool().add<kaanhbot::general::SetRobotOpMode>();

    plan_root->planPool().add<kaanhbot::config::MoveEa>();
    plan_root->planPool().add<kaanhbot::calibration::CalibT2P>();
    plan_root->planPool().add<kaanhbot::calibration::CalibW3P>();
    plan_root->planPool().add<kaanhbot::calibration::CalibT4P>();
    plan_root->planPool().add<kaanhbot::calibration::CalibT5P>();
    plan_root->planPool().add<kaanhbot::calibration::CalibT6P>();
    plan_root->planPool().add<kaanhbot::calibration::ManualSetTool>();
    plan_root->planPool().add<kaanhbot::calibration::GetTool>();
    plan_root->planPool().add<kaanhbot::calibration::ManualSetWobj>();
    plan_root->planPool().add<kaanhbot::calibration::GetWobj>();
    plan_root->planPool().add<kaanhbot::general::GetPe>();
    plan_root->planPool().add<kaanh::package::CCStop>();
    plan_root->planPool().add<kaanh::package::CalibDynPar>();
    plan_root->planPool().add<kaanh::package::CurrentGuidance>();
    plan_root->planPool().add<kaanh::multi_motion::JogJ>();
    plan_root->planPool().add<kaanh::multi_motion::JogC>();
    plan_root->planPool().add<kaanh::multi_motion::MoveAbsJ>();
    plan_root->planPool().add<kaanh::multi_motion::MoveL>();
    plan_root->planPool().add<kaanh::multi_motion::MoveC>();
    plan_root->planPool().add<kaanh::multi_motion::MoveLRel>();
    plan_root->planPool().add<kaanh::multi_motion::RePlay>();
    plan_root->planPool().add<kaanh::multi_motion::MoveArchP>();
    plan_root->planPool().add<kaanh::multi_motion::MoveArch>();
    plan_root->planPool().add<kaanh::multi_motion::ManualMoveAbsJ>();
    plan_root->planPool().add<kaanh::multi_motion::ManualMoveL>();
    plan_root->planPool().add<kaanhbot::motion::MoveLFind>();
    plan_root->planPool().add<kaanhbot::general::SetPdo>();
    plan_root->planPool().add<kaanhbot::general::GetPdo>();
    plan_root->planPool().add<kaanh::module::pauseProgram>();
    plan_root->planPool().add<kaanhbot::general::SetToolMode>();
    plan_root->planPool().add<kaanhbot::package::ConveyorFastCatch>();
    plan_root->planPool().add<kaanhbot::package::MoveConveyor>();
    plan_root->planPool().add<kaanhbot::package::SyncStop>();

    plan_root->planPool().add<kaanhbot::io::WaitDi>();
    plan_root->planPool().add<kaanhbot::functional::WaitMultiDi>();
    plan_root->planPool().add<kaanhbot::protocol::MtcpRoWaitBool>();
    plan_root->planPool().add<kaanhbot::protocol::MtcpWrWaitBool>();
    plan_root->planPool().add<kaanhbot::io::Pulse>();
    plan_root->planPool().add<kaanhbot::io::ManualSetDo>();
    plan_root->planPool().add<kaanhbot::io::SetDOPlan>();
    plan_root->planPool().add<kaanhbot::io::SetDOsPlan>();

    plan_root->planPool().add<robot::PoseTrajCli>();
    plan_root->planPool().add<CmdFtUnitTest>();
    plan_root->planPool().add<a10_tcp::A10PolicyTcpDriver>();
    plan_root->planPool().add<a10_tcp::A10PolicyTcpCliStop>();
    plan_root->planPool().add<a10_tcp::A10TeleopTcpDriver>();
    plan_root->planPool().add<a10_tcp::A10TeleopTcpCliStop>();

    return plan_root;
}
