#include "master_slave.hpp"
#include "a10_tcp_server.hpp"
#include "gravcomp.hpp"

#include <cmath>
#include <numeric>
#include <array>
#include <iostream>

using namespace std;
const double PI = 3.141592653589793;

//extern A10TcpServer *g_tcp_server;  // 在别处定义的全局 TCP 服务器指针

namespace master_slave
{
    struct MasterSlave::Imp {
        // Flag
        bool init = false;
        bool contact_check = false;

        // Arm1 (Leader) Parameters
        double arm1_init_force[6]{0};
        double arm1_p_vector[6]{0};
        double arm1_l_vector[6]{0};

        //Arm2
		double arm2_init_force[6]{ 0 };
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

        // Desired Pos, Vel, Force for Arm 1
        double arm1_x_d[6]{0};  // 记初始末端位姿
        double v_d[6]{0};       // 期望速度，默认 0
        double f_d[6]{0};       // 期望力，默认 0

        // Current Vel for Arm 1（位置+姿态）
        double v_c[6]{0};

        // Admittance Parameters for Arm 1
        double B[6]{ 1000,1000,1000,15,15,15 };
        double M[6]{ 10,10,10,10,5,5 };

        // Force Buffer 用于简单的滑动平均滤波
        std::array<double, 10> force_buffer[6]{};
        int buffer_index[6]{0};

        // Timer
        int start_count = 0;

        // 实际用于控制的“外力”，可以是模拟的，也可以将来接传感器
        double actual_force[6]{0};
    };

    auto MasterSlave::prepareNrt() -> void
    {
        for (auto &m : motorOptions())
            m = aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        // 真机时可以加载重力补偿参数
		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;

        mout() << "Master Slave Mode Prepared" << std::endl;
    }

    auto MasterSlave::executeRT() -> int
    {
        // dual transform modelbase into multimodel
        auto &dualArm = dynamic_cast<aris::dynamic::MultiModel &>(modelBase()[0]);
        auto &arm1 = dualArm.subModels().at(0); // Leader
        auto &arm2 = dualArm.subModels().at(1); // Follower

        auto &model_a1 = dynamic_cast<aris::dynamic::Model &>(arm1);
        auto &model_a2 = dynamic_cast<aris::dynamic::Model &>(arm2);

        auto &eeA1 = dynamic_cast<aris::dynamic::GeneralMotion &>(model_a1.generalMotionPool().at(0));

        static double tolerance = 0.0001;
        static double init_angle[12] =
            {0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
             0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0};

        static double trigger_force[6]{0.5, 0.5, 0.5, 0.001, 0.001, 0.001};
        static double max_force[6]{10, 10, 10, 5, 5, 5};

        GravComp gc;
        double current_angle[12]{0};
        double current_pos[6]{0};
        double filtered_force[6]{0};
        double transform_force[6]{0};
        double current_pm[16]{0};

        // --- Lambda: 读“力”（原来传感器，现在可以模拟） ---
        auto getForceData = [&](double *data_, int m_, bool init_)
        {
            int raw_force[6]{0};
            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;
                data_[i] = static_cast<double>(raw_force[i]) / 1000.0;
            }

            if (!init_)
            {
                // 第一次读，用来记录偏置
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                // 真机时减去初始偏置
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                        data_[i] = static_cast<double>(raw_force[i]) / 1000.0 - imp_->arm1_init_force[i];
                }
            }
        };

        // --- Lambda: 直接关节插补到目标姿态 ---
        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle_local[12]{0};
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
                current_angle_local[i] = controller()->motorPool()[i].targetPos();

            for (int i = 0; i < 12; i++)
            {
                if (current_angle_local[i] <= target_mp_[i] - move)
                    controller()->motorPool()[i].setTargetPos(current_angle_local[i] + move);
                else if (current_angle_local[i] >= target_mp_[i] + move)
                    controller()->motorPool()[i].setTargetPos(current_angle_local[i] - move);
            }
        };

        // --- Lambda: 检查电机是否到位 ---
        auto motorsPositionCheck = [](const double *current_sa_angle_, const double *target_pos_, size_t dim_)
        {
            for (int i = 0; i < static_cast<int>(dim_); i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                    return false;
            }
            return true;
        };

        // --- Lambda: 单臂 IK + 设电机 ---
        auto saMove = [&](double *pos_, aris::dynamic::Model &model_, int type_)
        {
            model_.setOutputPos(pos_);
            if (model_.inverseKinematics())
                throw std::runtime_error("Inverse Kinematics Position Failed!");

            double x_joint[6]{0};
            model_.getInputPos(x_joint);

            if (type_ == 0) // Arm1
            {
                for (std::size_t i = 0; i < 6; ++i)
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
            }
            else if (type_ == 1) // Arm2
            {
                for (std::size_t i = 0; i < 6; ++i)
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        // --- Lambda: 简单平均滤波 ---
        auto forceFilter = [&](double *actual_force_, double *filtered_force_)
        {
            for (int i = 0; i < 6; i++)
            {
                imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

                filtered_force_[i] = std::accumulate(
                                         imp_->force_buffer[i].begin(),
                                         imp_->force_buffer[i].end(),
                                         0.0) /
                                     10.0;
            }
        };

        // --- 每个周期读取当前实际关节位置 ---
        for (int i = 0; i < 12; i++)
            current_angle[i] = controller()->motorPool()[i].actualPos();

        // ===================== 初始化阶段 =====================
        if (!imp_->init)
        {
            dualArm.setInputPos(init_angle);
            if (dualArm.forwardKinematics())
                throw std::runtime_error("Forward Kinematics Position Failed!");

            daJointMove(init_angle);

            if (motorsPositionCheck(current_angle, init_angle, 12))
            {
                // 记录主臂初始末端位姿
                eeA1.getP(imp_->arm1_x_d);

                // 真机时这里读一次初始力偏置
                getForceData(imp_->arm1_init_force, 0, imp_->init);

                mout() << "Master Slave Init Done" << std::endl;
                imp_->init = true;
                imp_->start_count = count();
            }
        }
        // ================== 正常控制阶段 ==================
        else
        {
            // ------------ 1. 构造主臂“外力”（现在用模拟的，将来可用传感器） ------------
            // for (std::size_t i = 0; i < 6; ++i)
            //     imp_->actual_force[i] = 0.0;

            // int rel_count = count() - imp_->start_count;

            // // 举例：一段时间在 x 方向施加 -5N，之后在 z 方向 +5N
            // if (rel_count > 1000 && rel_count <= 3000)
            // {
            //     if (rel_count % 500 == 0)
            //         mout() << "Simulating Force X: -5N" << std::endl;
            //     imp_->actual_force[0] = -5.0;
            // }
            // else if (rel_count > 4000 && rel_count <= 6000)
            // {
            //     if (rel_count % 500 == 0)
            //         mout() << "Simulating Force Z: 5N" << std::endl;
            //     imp_->actual_force[2] = 5.0;
            // }

        //——————————如果以后接真实 F/T，可以替换为：——————————
            double current_force[6]{0};
            double comp_force[6]{0};
            eeA1.getMpm(current_pm);
            getForceData(current_force, 0, imp_->init);
            gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
            for (int i = 0; i < 6; ++i)
                imp_->actual_force[i] = comp_force[i] + current_force[i];

            // ------------ 2. 导纳控制：用力拖动主臂 ------------
            {
                double current_vel[6]{0};
                eeA1.getP(current_pos);
                eeA1.getV(current_vel);
                eeA1.getMpm(current_pm);

                // 力死区 + 限幅（和 ForceDrag 一样的思路）
                for (int i = 0; i < 6; ++i)
                {
                    double &f = imp_->actual_force[i];
                    if (std::fabs(f) < trigger_force[i]) f = 0.0;
                    if (f >  max_force[i]) f =  max_force[i];
                    if (f < -max_force[i]) f = -max_force[i];
                }

                // 滤波
                forceFilter(imp_->actual_force, filtered_force);

                //将力数据坐标变换到Arm1基坐标系下
                transform_force[0] = filtered_force[2];
                transform_force[1] = -filtered_force[1];
                transform_force[2] = filtered_force[0];
                transform_force[3] = filtered_force[5];
                transform_force[4] = -filtered_force[4];
                transform_force[5] = filtered_force[3];

                // ====== 导纳控制器（保持和 ForceDrag 行为一致：用 actual_force 驱动） ======
                double acc[3]{0};
                double ome[3]{0};
                double dx[3]{0};
                double dth[3]{0};
                double dt = 0.001;

                // 位置部分
                for (int i = 0; i < 3; ++i)
                {
                    // da = (Fd - Fe - B*(v - vd)) / M   （这里不加弹簧项）
                    acc[i] = (-imp_->f_d[i] + imp_->actual_force[i]
                              - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) /
                             imp_->M[i];
                }
                for (int i = 0; i < 3; ++i)
                {
                    imp_->v_c[i] += acc[i] * dt;
                    dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                    current_pos[i] += dx[i];
                }

                // 姿态部分（角速度）
                for (int i = 0; i < 3; ++i)
                {
                    ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3]
                              - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) /
                             imp_->M[i + 3];
                }
                for (int i = 0; i < 3; ++i)
                {
                    imp_->v_c[i + 3] += ome[i] * dt;
                    dth[i] = imp_->v_c[i + 3] * dt;
                }

                double drm[9]{0};
                double rm_target[9]{0};
                double rm_c[9]{0};

                // 由“角增量”算目标姿态
                aris::dynamic::s_ra2rm(dth, drm);
                aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");
                aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);
                aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                // 通过 IK 把末端位姿映射回关节角
                saMove(current_pos, model_a1, 0);

                if (count() % 100 == 0)
                {
                    mout() << "EE1 pos: "
                           << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                           << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }
            }

            // ------------ 3. 从臂：通过 TCP 跟随 Leader 的目标关节 ------------
            // if (g_tcp_server)
            // {
            //     std::vector<double> target_q = g_tcp_server->get_latest_q();

            //     // 这里假设 target_q 是从 Leader 端发来的 6 关节角（或者你自己约定的格式）
            //     if (target_q.size() >= 6)
            //     {
            //         for (int i = 0; i < 6; ++i)
            //         {
            //             // 从臂电机索引 6~11
            //             controller()->motorPool()[i + 6].setTargetPos(-target_q[i]);
            //         }
            //     }
            // }
        }

        // 让这个 Plan 运行一段时间后自动结束
        return 15000 - count();
    }

    MasterSlave::MasterSlave(const std::string &name)
    {
        imp_.reset(new Imp);

        aris::core::fromXmlString(command(),
                                  "<Command name=\"master_slave\">"
                                  "</Command>");
    }

    MasterSlave::~MasterSlave() = default;

    ARIS_REGISTRATION
    {
        aris::core::class_<MasterSlave>("MasterSlave")
            .inherit<aris::plan::Plan>();
    }
}
