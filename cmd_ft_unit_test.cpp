#include "cmd_ft_unit_test.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include <aris/core/log.hpp>

namespace {
constexpr int kFtDataSize = 6;
}

struct CmdFtUnitTest::Imp {
    int ft_id = 8;
    double dt = 0.002;
    double duration = 10.0;
    int run_count = 0;
    bool log_started = false;

    double ft_raw[kFtDataSize]{0.0};
    int raw_force_int[kFtDataSize]{0};

    auto startLogIfNeeded(CmdFtUnitTest* self) -> void {
        if (log_started) {
            return;
        }

        std::string file_name =
            std::string("ft_unit_test_") + aris::core::logFileTimeFormat(std::chrono::system_clock::now());
        self->master()->logFileRawName(file_name.c_str());
        self->mout() << "[ft_unit] log file: " << file_name << std::endl;

        self->lout() << std::fixed << std::setprecision(15);
        self->lout()
            << "rt_time" << ","
            << "count" << ","
            << "elapsed_time" << ","
            << "read_ok" << ","
            << "raw_int_fx" << ","
            << "raw_int_fy" << ","
            << "raw_int_fz" << ","
            << "raw_int_tx" << ","
            << "raw_int_ty" << ","
            << "raw_int_tz" << ","
            << "raw_fx" << ","
            << "raw_fy" << ","
            << "raw_fz" << ","
            << "raw_tx" << ","
            << "raw_ty" << ","
            << "raw_tz"
            << "\n";

        log_started = true;
    }

    auto readRawFt(CmdFtUnitTest* self) -> bool {
        auto* ec = self->ecMaster();
        if (ec == nullptr) {
            return false;
        }
        auto& slaves = ec->slavePool();
        if (ft_id < 0 || static_cast<std::size_t>(ft_id) >= slaves.size()) {
            return false;
        }

        bool ok = true;
        for (int i = 0; i < kFtDataSize; ++i) {
            raw_force_int[i] = 0;
            if (slaves[static_cast<std::size_t>(ft_id)].readPdo(0x6020, 0x01 + i, raw_force_int + i, 32)) {
                ok = false;
            }
            ft_raw[i] = static_cast<double>(raw_force_int[i]) / 1000.0;
        }

        return ok;
    }
};

auto CmdFtUnitTest::prepareNrt() -> void {
    std::cout << "\n================ [FT UNIT TEST] 初始化 ================\n";

    for (auto& m : motorOptions()) {
        m = NOT_CHECK_VEL_MAX
          | NOT_CHECK_VEL_MIN
          | NOT_CHECK_VEL_CONTINUOUS
          | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER
          | NOT_CHECK_ENABLE;
    }

    imp_->ft_id = int32Param("ft_id");
    imp_->duration = doubleParam("time");
    imp_->run_count = 0;
    imp_->log_started = false;

    std::cout << "ft_id    : " << imp_->ft_id << std::endl;
    std::cout << "duration : " << imp_->duration << " s" << std::endl;
    std::cout << "log mode : every control cycle" << std::endl;

    if (auto* ec = ecMaster()) {
        const std::size_t n = ec->slavePool().size();
        std::cout << "ec slaves: " << n << " (valid ft_id: 0 ~ " << (n == 0 ? 0 : n - 1) << ")\n";
        if (imp_->ft_id < 0 || static_cast<std::size_t>(imp_->ft_id) >= n) {
            std::cout << "[ft_unit] 警告: ft_id 超出从站范围，executeRT 将不读 PDO（请设正确 ft_id）\n";
        }
    } else {
        std::cout << "[ft_unit] 警告: ecMaster() 为空（当前非 EtherCAT Master），无法读六维力 PDO\n";
    }

    std::cout << "======================================================\n" << std::endl;

    imp_->startLogIfNeeded(this);
}

auto CmdFtUnitTest::executeRT() -> int {
    ++imp_->run_count;

    if (imp_->duration > 0.0 && imp_->run_count * imp_->dt >= imp_->duration) {
        mout() << "[ft_unit] timeout reached: " << imp_->duration << " s" << std::endl;
        return 0;
    }

    const bool ok = imp_->readRawFt(this);
    const std::int64_t rt_time = aris::control::aris_rt_timer_read();
    const double elapsed_time = imp_->run_count * imp_->dt;

    lout() << rt_time << ","
           << count() << ","
           << elapsed_time << ","
           << (ok ? 1 : 0) << ","
           << imp_->raw_force_int[0] << ","
           << imp_->raw_force_int[1] << ","
           << imp_->raw_force_int[2] << ","
           << imp_->raw_force_int[3] << ","
           << imp_->raw_force_int[4] << ","
           << imp_->raw_force_int[5] << ","
           << imp_->ft_raw[0] << ","
           << imp_->ft_raw[1] << ","
           << imp_->ft_raw[2] << ","
           << imp_->ft_raw[3] << ","
           << imp_->ft_raw[4] << ","
           << imp_->ft_raw[5]
           << "\n";

    if (count() % 500 == 0) {
        if (ok) {
            mout() << "[ft_unit] raw=("
                   << imp_->ft_raw[0] << ", "
                   << imp_->ft_raw[1] << ", "
                   << imp_->ft_raw[2] << ", "
                   << imp_->ft_raw[3] << ", "
                   << imp_->ft_raw[4] << ", "
                   << imp_->ft_raw[5] << ")"
                   << std::endl;
        } else {
            mout() << "[ft_unit] read failed at count=" << count() << std::endl;
        }
    }

    return 1;
}

auto CmdFtUnitTest::collectNrt() -> void {
    std::cout << "\n[ft_unit] 完成" << std::endl;
    std::cout << "总运行周期: " << imp_->run_count << std::endl;
    std::cout << "总运行时间: " << imp_->run_count * imp_->dt << " s" << std::endl;
}

CmdFtUnitTest::CmdFtUnitTest(const std::string& name) : imp_(new Imp) {
    aris::core::fromXmlString(command(),
        "<Command name=\"ft_unit\">"
        "  <GroupParam>"
        "    <Param name=\"ft_id\" default=\"8\" abbreviation=\"f\"/>"
        "    <Param name=\"time\" default=\"10.0\" abbreviation=\"t\"/>"
        "  </GroupParam>"
        "</Command>");
}

CmdFtUnitTest::~CmdFtUnitTest() = default;

KAANH_DEFINE_BIG_FOUR_CPP(CmdFtUnitTest);

ARIS_REGISTRATION {
    aris::core::class_<CmdFtUnitTest>("CmdFtUnitTest").inherit<aris::plan::Plan>();
}
