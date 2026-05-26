#include "rivetcomd.hpp"
#include "robot.hpp"
#include "gravcomp.hpp"
#include "searchDir.hpp"
#include <array>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>  // 包含 std::pair
#include <string>
#include <sstream>

using namespace std;

namespace rivet
{

    struct RivetInit::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

    };
    auto RivetInit::prepareNrt()->void {

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


    }
    auto RivetInit::executeRT()->int {

        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


        GravComp gc;

        static double d_pos = 0.00001;
        static double tolerance = 0.00007;

        double current_angle[12]{ 0 };
        double current_arm1_angle[6]{ 0 };
        double current_arm2_angle[6]{ 0 };


        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double actual_force[6]{ 0 };
        double transform_force[6]{ 0 };



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00007;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        //single arm move 1-->white 2-->blue
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };


        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }

        std::copy(current_angle, current_angle + 6, current_arm1_angle);
        std::copy(current_angle + 6, current_angle + 12, current_arm2_angle);




        double assem_pos_a1[6]{ 0.615, 0.125, 0.256, PI / 2, -PI / 2, PI / 2 };
        double assem_pos_a2[6]{ -0.600, 0.042, 0.365, PI, 0, PI };
        double init_angle[12]{0};

        model_a1.setOutputPos(assem_pos_a1);
        if(model_a1.inverseKinematics())
        {
            mout()<<"Arm1 Error"<<std::endl;
        }
        
        model_a2.setOutputPos(assem_pos_a2);
        if(model_a2.inverseKinematics())
        {
            mout()<<"Arm2 Error"<<std::endl;
        }

        dualArm.getInputPos(init_angle);

        if(count()%1000 == 0)
        {
            mout()<<"current angle: "<<current_angle[0]<<'\t'<<current_angle[1]<<'\t'<<current_angle[2]<<'\t'
                    <<current_angle[3]<<'\t'<<current_angle[4]<<'\t'<<current_angle[5]
                    <<current_angle[6]<<'\t'<<current_angle[7]<<'\t'<<current_angle[8]<<'\t'
                    <<current_angle[9]<<'\t'<<current_angle[10]<<'\t'<<current_angle[11]<<std::endl;
        }

        daJointMove(init_angle);

        if(motorsPositionCheck(current_angle, init_angle, 12))
        {
            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);

            gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

            mout()<<"Init Complete"<<std::endl;

            imp_->init = true;
            return 0;
        }



        if(count() == 35000)
        {
            mout()<<"Over Time"<<std::endl;
        }

        return 35000 - count();




    }
    RivetInit::RivetInit(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"r_init\"/>");
    }
    RivetInit::~RivetInit() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(RivetInit)

    struct RivetStart::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;


        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};
        double arm1_temp_force2[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        double arm2_temp_force[6]{0};
        double arm2_temp_force2[6]{0};

        //Time
        int init_count = 0;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };


    };
    auto RivetStart::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto RivetStart::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00007;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};

        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[0];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = -actual_force_[2];

                transform_force_[3] = -actual_force_[3];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = -actual_force_[5];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {
            double assem_pos_a1[6]{ 0.615, 0.125, 0.256, PI / 2, -PI / 2, PI / 2 };
            double assem_pos_a2[6]{ -0.600, 0.042, 0.365, PI, 0, PI };
            double init_angle[12]{0};

            model_a1.setOutputPos(assem_pos_a1);
            if(model_a1.inverseKinematics())
            {
                mout()<<"Arm1 Error"<<std::endl;
            }
            
            model_a2.setOutputPos(assem_pos_a2);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Arm2 Error"<<std::endl;
            }

            dualArm.getInputPos(init_angle);

            if(count()%1000 == 0)
            {
                mout()<<"current angle: "<<current_angle[0]<<'\t'<<current_angle[1]<<'\t'<<current_angle[2]<<'\t'
                        <<current_angle[3]<<'\t'<<current_angle[4]<<'\t'<<current_angle[5]
                        <<current_angle[6]<<'\t'<<current_angle[7]<<'\t'<<current_angle[8]<<'\t'
                        <<current_angle[9]<<'\t'<<current_angle[10]<<'\t'<<current_angle[11]<<std::endl;
            }

            daJointMove(init_angle);

            if(motorsPositionCheck(current_angle, init_angle, 12))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
                imp_->init_count = count();
                //return 0;
            }
        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            //forceDeadZone(arm2_filtered_force, arm2_dead_zone);


            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);

            for (size_t i = 0; i < 6; i++)
            {

                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }

            double rivet_pos_a1[6]{ 0.800, 0.125, 0.256, PI / 2, -PI / 2, PI / 2 };
            double rivet_pos_a2[6]{ -0.824594, 0.098932, 0.33, PI, 0, PI };
            double rivet_start_angle[12]{0};

            model_a1.setOutputPos(rivet_pos_a1);
            if(model_a1.inverseKinematics())
            {
                mout()<<"Arm1 Error"<<std::endl;
            }
            
            model_a2.setOutputPos(rivet_pos_a2);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Arm2 Error"<<std::endl;
            }

            dualArm.getInputPos(rivet_start_angle);

            if(count()%1000 == 0)
            {
                mout()<<"current angle: "<<current_angle[0]<<'\t'<<current_angle[1]<<'\t'<<current_angle[2]<<'\t'
                        <<current_angle[3]<<'\t'<<current_angle[4]<<'\t'<<current_angle[5]
                        <<current_angle[6]<<'\t'<<current_angle[7]<<'\t'<<current_angle[8]<<'\t'
                        <<current_angle[9]<<'\t'<<current_angle[10]<<'\t'<<current_angle[11]<<std::endl;
            }

            daJointMove(rivet_start_angle);

            if(motorsPositionCheck(current_angle, rivet_start_angle, 12))
            {

                mout()<<"Start Pos Complete"<<std::endl;

                return 0;
            }

        }


        return 100000 - count();
    }
    RivetStart::RivetStart(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"r_start\"/>");
    }
    RivetStart::~RivetStart() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(RivetStart)

    struct Arm2Init::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Switch Start Pos
        int p_;

    };
    auto Arm2Init::prepareNrt()->void {

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


    }
    auto Arm2Init::executeRT()->int {

        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


        GravComp gc;

        static double d_pos = 0.00001;
        static double tolerance = 0.00005;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double actual_force[6]{ 0 };
        double transform_force[6]{ 0 };

        imp_->p_ = int32Param("pos");



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        //single arm move 1-->white 2-->blue
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };


        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);


        double assem_pos[6]{0};
        
        if(imp_->p_ == 0)
        {
            double temp_pos[6]{-0.824594, 0.098932, 0.33, PI, 0, PI};
            std::copy(temp_pos, temp_pos+6, assem_pos);
        }
        else if(imp_->p_ == 1)
        {
            double temp_pos[6]{-0.622144, -0.206780, 0.08, PI, 0, PI};
            std::copy(temp_pos, temp_pos+6, assem_pos);
        }
        else if(imp_->p_ == 2)
        {
            double temp_pos[6]{-0.622925, -0.102052, 0.08, PI, 0, PI};
            std::copy(temp_pos, temp_pos+6, assem_pos);
        }
        else
        {
            mout()<<"Wrong Input Pos"<<std::endl;
            return 0;
        }


        //double assem_pos[6]{ -0.824594, 0.098932, 0.33, PI, 0, PI };
        double init_angle[6]{0};

        model_a2.setOutputPos(assem_pos);
        if(model_a2.inverseKinematics())
        {
            mout()<<"Error"<<std::endl;
        }

        model_a2.getInputPos(init_angle);

        if(count()%1000 == 0)
        {
            mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                    <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
        }

        saJointMove(init_angle, 1);

        if(motorsPositionCheck(current_sa_angle, init_angle, 6))
        {
            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);

            gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

            mout()<<"Init Complete"<<std::endl;

            imp_->init = true;
            return 0;
        }



        if(count() == 20000)
        {
            mout()<<"Over Time"<<std::endl;
        }

        return 20000 - count();




    }
    Arm2Init::Arm2Init(const std::string& name)
    {
        aris::core::fromXmlString(command(),
        "<Command name=\"2_back\">"
         "	<GroupParam>"
         "	<Param name=\"pos\" default=\"0\" abbreviation=\"p\"/>"
         "	</GroupParam>"
         "</Command>");
    }
    Arm2Init::~Arm2Init() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Arm2Init)

    struct RivetOut::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };


        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double B[6]{ 3500,3500,2500,0,0,0 };
        double M[6]{ 100,100,100,0,0,0 };

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Deadzone
        double deadzone[6]{3,3,3,0,0,0};

        //Desired Force
        double f_d[6]{0,0,0,0,0,0};
        double v_d[6]{0};

    };
    auto RivetOut::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
        gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
        mout()<<"Load Init Force"<<std::endl;
    }
    auto RivetOut::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


        GravComp gc;

        static double d_pos = 0.00001;

        double current_angle[12]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double actual_force[6]{ 0 };
        double transform_force[6]{ 0 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };




        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[0];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = -actual_force_[2];

                transform_force_[3] = -actual_force_[3];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = -actual_force_[5];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };


        for(int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }

        if(count() == 1)
        {
            dualArm.setInputPos(current_angle);
            if(dualArm.forwardKinematics()){mout()<<"Error"<<std::endl;}
            mout()<<"Init"<<std::endl;
        }

        double acc[3]{0};
        double dx[3]{0};

        double dt = 0.001;

        eeA2.getP(current_pos);
        eeA2.getMpm(current_pm);

        getForceData(actual_force, 1, true);
        gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
        for (size_t i = 0; i < 6; i++)
        {
           comp_force[i] = actual_force[i] + comp_force[i];
        }

        forceTransform(comp_force, transform_force, 1);

        //Safety Check
        for (size_t i = 0; i < 3; i++)
        {
            if (abs(transform_force[i]) > 30)
            {
                mout() << "Emergency Brake" << std::endl;
                return 0;
            }
        }

        if (count() % 100 == 0)
        {
            mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t'<< "pos: " << current_pos[1] << '\t' << current_pos[2] <<std::endl;
        }


        current_pos[2] += d_pos;


        forceUpperLimit(transform_force, limit_area);
        forceDeadZone(transform_force, imp_->deadzone);


        //Impedence Controller
        for (int i = 0; i < 2; i++)
        {
            // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
            acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
        }


        for (int i = 0; i < 2; i++)
        {
            imp_->v_c[i] += acc[i] * dt;
            velDeadZone(imp_->v_c[i], max_vel[i]);
            dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
            current_pos[i] = dx[i] + current_pos[i];

        }

        saMove(current_pos, model_a2, 1);


        return 5000 - count();
    }
    RivetOut::RivetOut(const std::string& name)
    {
       aris::core::fromXmlString(command(),
            "<Command name=\"r_out\"/>");
    }
    RivetOut::~RivetOut() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(RivetOut)

    struct RivetSearch::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;
        bool phase6 = false;

        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};
        double arm1_temp_force2[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        double arm2_temp_force[6]{0};
        double arm2_temp_force2[6]{0};

        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };

        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase2_fd[6]{ 0 };
        double phase3_fd[6]{ 0,0,5,0,0,0 };
        double phase4_fd[6]{ 0,0,5,0,0,0 };
        double phase5_fd[6]{ 0,0,6,0,0,0 };

        double phase6_fd[6]{ 0,0,0,0,0,0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        //double K[6]{ 100,100,100,15,15,15 };
        double phase3_B[6]{ 1500,1500,8000,3,3,3 };
        double phase3_M[6]{ 100,100,100,2,2,2 };

        //Search
        double phase4_B[6]{ 2500,2500,3500,3,3,3 };
        double phase4_M[6]{ 100,100,100,2,2,2 };

        double phase5_B[6]{ 3500,3500,30000,0,0,0 };
        double phase5_M[6]{ 100,100,400,0,0,0 };

        double phase6_B[6]{ 3500,3500,2500,0,0,0  };
        double phase6_M[6]{ 100,100,100,0,0,0 };

        //Force Counter
        int allign_count = 0;
        int success_count = 0;

        //Counter for force comp
        int start_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        double current_pos_checkek[6] = {0};


        //Switch Angle
        double x_;
        double y_;

        //Switch Point
        int p_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Parameters for Pos Recoginistion
        double a_y = -0.0567;
        double b_y = -0.9679;
        double c_y = 0.0010;

        double a_z = 0.9745;
        double b_z = -0.0535;
        double c_z = 0.0063;

        double hole_pos[6]{0};
        double hole_angle[6]{0};


        //Dead Zone
        double p4_deadzone[6]{1.0,1.0,1.0,0,0,0};
        double p5_deadzone[6]{2.5,2.5,0.1,0,0,0};
        double p6_deadzone[6]{3,3,3,0,0,0};

        //Search Parameter
		double px[31] = {0, 0.00025, 0.0005, 0.00075, 0.001, 0.00125, 0.0015, 0.00175, 0.002, 0.00225, 0.0025, 
        0.00275, 0.003, 0.00325, 0.0035, 0.00375, 0.004, 0.00425, 0.0045, 0.00475, 	
		0.005, 0.00525, 0.0055, 0.00575, 0.006, 0.00625, 0.0065, 0.00675, 0.007, 0.00725, 0.0075};

		double py[31] = {0, -0.0002097, 0.0004195, -0.0006293, 0.0008391, -0.0010488, 0.0012586, -0.001468, 0.001678, 
        -0.001888, 0.0020977, -0.002308, 0.002517, -0.0027271, 0.002937, -0.003147, 0.003356	
		-0.003566, 0.003776, -0.003986,	0.004195, -0.004405, 0.004615, 	-0.004815, 0.00450, -0.004146, 0.003742, -0.003269, 0.002693, -0.001920, 0};

		double force_direction[30]{0};

		
		double each_count[30] = {213, 389, 589, 795, 1002, 1211, 1419, 1628, 1837, 2047, 2256, 2466, 2675, 2885, 3094, 3304, 3514, 3723,	 
		3933, 4143, 4352, 4562, 4767, 4709, 4375, 3996, 3558, 3033, 2360, 1018};

        double pos_error = 0;

        //Counter
        int search_start_count = 0;
        int search_counter = 0;

        int total_search_count = 0;

        //Time Modify
        double avg_trans_x_force = 0;

        double temp_trans_x_force = 0;

        int trans_force_counter = 0;

        double theta;
        double search_pos[2]{0};

        double search_start_pos[6]{0};

        //Input Direction
        double dy_;
        double dx_;

        //Documented Depth
        double depth;
        double y_movement;
        double x_movement;

        //Back
        double back_count;

        //Side Movement Jammed Check
        double check_disp = 0;
        int jammed_count = 0;
        int jammed_check_count = 0;





    };
    auto RivetSearch::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto RivetSearch::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);

        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};

        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};

        double net_disp = 0;


        imp_->x_ = doubleParam("x_degree");

        imp_->y_ = doubleParam("y_degree");

        imp_->p_ = int32Param("point");

        imp_->dy_ = doubleParam("search_y");

        imp_->dx_ = doubleParam("search_x");


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[0];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = -actual_force_[2];

                transform_force_[3] = -actual_force_[3];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = -actual_force_[5];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_, int num_)
        {

            bool isValid = true;
            for (int i = 0; i < num_; i++)
            {
                if (abs(current_force_[i]) > force_check_[i])
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 50 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() <<'\t'<<"Pos Error: "<<imp_->pos_error<< std::endl;
                    if (imp_->success_count >= count_)
                    {
                        imp_->success_count = 0;
                        imp_->allign_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        auto boltJammedCheck = [&](double net_disp_, int count_)
        {
            if(count()%500 == 0)
            {
                imp_->check_disp = net_disp_;
            }

            bool isValid = true;

            if (abs(imp_->check_disp - net_disp_) >= 0.0001)
            {
                isValid = false;
            }

            if(isValid)
            {
                if(imp_->jammed_count == 0)
                {
                    imp_->jammed_count = count();
                }

                if((count() - imp_->jammed_count) % 100 == 0)
                {
                    imp_->jammed_check_count++;
                    mout() << "Check " << imp_->jammed_check_count << '\t' << "Current Count: " << count() << '\t' << "d_disp: " << abs(imp_->check_disp - net_disp_)<< std::endl;
                    if(imp_->jammed_check_count >= count_)
                    {
                        imp_->jammed_count = 0;
                        imp_->jammed_check_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->jammed_check_count = 0;
            }

            return false;
        };


        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }




        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {
            double assem_pos[6]{ -0.824594, 0.098932, 0.30, PI, 0, PI };
            double init_angle[6]{0};

            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a2.getInputPos(init_angle);

            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }

            saJointMove(init_angle, 1);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
            }
        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            //forceDeadZone(arm2_filtered_force, arm2_dead_zone);


            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);

            for (size_t i = 0; i < 6; i++)
            {

                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }


            imp_->depth = imp_->search_start_pos[2] - arm2_current_pos[2];
            imp_->y_movement = imp_->search_start_pos[1] - arm2_current_pos[1];
            imp_->x_movement = imp_->search_start_pos[0] - arm2_current_pos[0];
            


            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ -0.824594, 0.098932, 0.30, PI, 0, PI  };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate displacment

                double rotate_angle[3]{ (imp_->x_ )* 2 * PI / 360, (imp_->y_)* 2 * PI / 360, 0 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");


                std::array<std::array<double, 3>, 10> points =
                {
                    {
                        {0.005, 0, 0},
                        {-0.005, 0, 0},
                        {0, 0.005, 0},
                        {0, -0.005, 0},
                        {0.0035, 0.0035, 0},
                        {-0.0035, -0.0035, 0},
                        {0.0035, -0.0035, 0},
                        {-0.0035, 0.0035, 0},
                        {0.0035, 0.002, 0},
                        {-0.0035, 0.0015, 0}
                    }
                };


                if(imp_->p_ < 0 || imp_->p_ >= points.size())
                {
                    mout()<<"Error Points Index"<<std::endl;
                    return 0;
                }

                for(size_t i = 0; i < 3; i++)
                {
                    assem_pos[i] += points[imp_->p_][i];
                }


                eeA2.setP(assem_pos);


                if (model_a2.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a2.getInputPos(assem_angle);

                saJointMove(assem_angle, 1);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {

                    imp_->phase1 = true;
                    imp_->start_count = count();

                    mout()<<"Current Angle: "<< rotate_angle[0] <<'\t'<< rotate_angle[1] <<'\t' << rotate_angle[2]<<std::endl;
                    mout()<<"Current Pos: "<< assem_pos[0] <<'\t'<< assem_pos[1] <<'\t' << assem_pos[2]<<std::endl;
                    mout() << "Assembly Start !" << std::endl;
                }

            }
            //Phase 2 Contact, Have Certain Position Adjustment
            else if (imp_->phase1 && !imp_->phase2)
            {

                if(count() == imp_->start_count + 2000)
                {


                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;


                }


                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);

                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                }


                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase2 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        break;


                    }

                }
                if (!imp_->phase2)
                {
                    arm2_current_pos[2] -= 0.000003;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Stable Contact
            else if (imp_->phase2 && !imp_->phase3)
            {

                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;


                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 15.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 5))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    eeA2.getP(imp_->search_start_pos);
                    imp_->theta = std::atan2(imp_->dy_, imp_->dx_);

                    for(int i = 0; i < 30; i++)
                    {
                        imp_->force_direction[i] = std::atan2((imp_->py[i+1] - imp_->py[i]), (imp_->px[i+1] - imp_->px[i])) - imp_->theta;
                    }

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    // master()->logFileRawName("Search_contact");
                    master()->logFileRawName(std::string("/home/kaanh/Desktop/kaanhbin/search_test/searchTest_" + aris::core::logFileTimeFormat(std::chrono::system_clock::now())).c_str());
                    imp_->total_search_count = count();

                    //return 0;

                }
                else
                {

                    if(arm2_final_force[2] >= (imp_->phase3_fd[2] - 0.01) && arm2_final_force[2] <= (imp_->phase3_fd[2] + 2))
                    {
                        arm2_final_force[2] = imp_->phase3_fd[2];
                    }


                    //Impedence Controller
                    for (int i = 2; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_final_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 2; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 4 Fine Search
            else if (imp_->phase3 && !imp_->phase4)
            {
                double arm2_check_force[6]{0,0,0,0,0,0};

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 15.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }



                imp_->pos_error = imp_->search_start_pos[0] - arm2_current_pos[0];
                net_disp = sqrt(imp_->x_movement*imp_->x_movement + imp_->y_movement*imp_->y_movement);


                if (count()%25 == 0)
                {
                   lout()<< arm2_final_force[0] <<'\t'<< arm2_final_force[1] <<'\t'<<arm2_final_force[2] 
                   <<'\t'<<arm2_final_force[3]<<'\t'<<arm2_final_force[4]<<'\t'<<arm2_final_force[5]
                   <<'\t'<<imp_->depth<<'\t'<< imp_->y_movement <<'\t'<< imp_->x_movement <<'\t'<< count() << std::endl;
                }

                //0.00015
                if (boltJammedCheck(net_disp, 15))
                {
                    imp_->phase4 = true;
                    mout() << "Pos 4 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    mout()<<"Total Search Time: "<< (count() - imp_->total_search_count) << std::endl;

                    //lout() << "[Stage 4 End]" << '\t' <<count()<<std::endl; 

                    //return 0;

                }
                else
                {

                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p4_deadzone);

                    if(arm2_final_force[2] >= (imp_->phase4_fd[2] - 0.01) && arm2_final_force[2] <= (imp_->phase4_fd[2] + 3.5))
                    {
                        arm2_final_force[2] = imp_->phase4_fd[2];
                    }


                    double real_xy_force[2]{0};

                    for(int i = 0; i < 2; i++)
                    {
                        real_xy_force[i] = arm2_transform_force[i];
                    }

                    double expected_force[2]{-5.0,0};

                    eeA2.getP(arm2_current_pos);


                    if(imp_->search_counter < 30)
                    {
                        //Only xy movment

                        //int next_counter = round((1-(abs(imp_->avg_trans_x_force / 6.0))) * imp_->each_count[imp_->search_counter]);


                        imp_->force_direction[imp_->search_counter] = (1 - abs(imp_->avg_trans_x_force / 18.0))*std::atan2((imp_->py[imp_->search_counter+1] - imp_->py[imp_->search_counter]), (imp_->px[imp_->search_counter+1] - imp_->px[imp_->search_counter])) - imp_->theta;


                        if(imp_->search_start_count <= imp_->each_count[imp_->search_counter])
                        {

                            double trans_z[4] = {cos(imp_->force_direction[imp_->search_counter]), -sin(imp_->force_direction[imp_->search_counter]),
                                                sin(imp_->force_direction[imp_->search_counter]), cos(imp_->force_direction[imp_->search_counter])};

                            double inv_z[4] = {cos(imp_->force_direction[imp_->search_counter]), sin(imp_->force_direction[imp_->search_counter]),
                                                -sin(imp_->force_direction[imp_->search_counter]), cos(imp_->force_direction[imp_->search_counter])};

                            double acc[2]{ 0 };
                            double dx[2]{ 0 };
                            double dt = 0.001;

                            double trans_dx[2]{0};
                            double trans_xy_force[2]{0};

                            aris::dynamic::s_mm(2,1,2,trans_z,real_xy_force,trans_xy_force);

                            if(trans_xy_force[0] >= (expected_force[0] - 1.5) && trans_xy_force[0] <= (expected_force[0] + 1.5))
                            {
                                trans_xy_force[0] = expected_force[0];
                            }


                            if(imp_->search_start_count >= round(2.0 * imp_->each_count[imp_->search_counter] / 3.0) && imp_->search_start_count <= imp_->each_count[imp_->search_counter])
                            {
                                imp_->trans_force_counter ++;
                                imp_->temp_trans_x_force += trans_xy_force[0];
                            }
                            if(imp_->search_start_count == imp_->each_count[imp_->search_counter])
                            {
                                imp_->avg_trans_x_force = imp_->temp_trans_x_force / imp_->trans_force_counter;
                            }

                            //Impedence Controller for X Y Movement
                            for (int i = 0; i < 2; i++)
                            {
                                // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                                acc[i] = (-expected_force[i] + trans_xy_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                            }


                            for (int i = 0; i < 2; i++)
                            {
                                imp_->v_c[i] += acc[i] * dt;
                                velDeadZone(imp_->v_c[i], max_vel[i]);
                                dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;

                            }

                            aris::dynamic::s_mm(2,1,2,inv_z,dx,trans_dx);

                            double acc_x = 0;
                            double dx_x = 0;

                            //Impedence Controller for Z Only
                            acc_x = (-imp_->phase4_fd[2] + arm2_transform_force[2] - imp_->phase4_B[2] * (imp_->v_c[2] - imp_->v_d[2])) / imp_->phase4_M[2];
                            imp_->v_c[2] += acc_x * dt;
                            velDeadZone(imp_->v_c[2], max_vel[2]);
                            dx_x = imp_->v_c[2] * dt + acc_x * dt * dt;

                            arm2_current_pos[2] += dx_x;

                            arm2_current_pos[0] += trans_dx[0];
                            arm2_current_pos[1] += trans_dx[1];

                            imp_->search_start_count++;


                            if(count() % 100 == 0)
                            {
    //                                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<
    //                                '\t'<<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t'
    //                                     <<"Trans_force: "<<trans_yz_force[0]<<'\t'<<trans_yz_force[1]<<std::endl;


//                                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<'\t'
//                                     <<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t'
//                                    << "Trans_force:"<<'\t'<<trans_yz_force[0]<<'\t'<<trans_yz_force[1] << '\t'
//                                    << "Counter:"<<'\t'<<imp_->search_counter<<'\t'
//                                    << "Deg:"<<'\t'<< 57.296 * imp_->force_direction[imp_->search_counter] <<std::endl;


                                mout()<<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t' <<std::endl;
                            }


                        }
                        else
                        {

                            for(int i = 0; i < 2; i++)
                            {
                                imp_->v_c[i] = 0;
                            }
                            imp_->search_counter++;
                            imp_->search_start_count = 0;
                            mout()<<"Search Counter: "<<imp_->search_counter<<std::endl;
                        }


                        saMove(arm2_current_pos, model_a2, 1);

                    }
                    else
                    {
                        mout()<<"Finish"<<std::endl;
                        return 0;
                    }
                }

            }
            //Phase 5 Maintain 2 Points Contact
            else if (imp_->phase4 && !imp_->phase5)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                // if (count()%25 == 0)
                // {
                //    lout()<< arm2_final_force[0] <<'\t'<< arm2_final_force[1] <<'\t'<<arm2_final_force[2] 
                //    <<'\t'<<arm2_final_force[3]<<'\t'<<arm2_final_force[4]<<'\t'<<arm2_final_force[5]
                //    <<'\t'<<imp_->depth<<'\t'<< imp_->y_movement <<'\t'<< imp_->x_movement <<'\t'<< count() << std::endl;
                // }
                
                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase5 = true;
                    mout() << "Pos 5 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    //lout() << "[Stage 5 End]" << '\t' <<count()<<std::endl; 

                    imp_->back_count = count();

                    //return 0;

                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p5_deadzone);

                    if(arm2_final_force[2] >= (imp_->phase5_fd[2] - 1) && arm2_final_force[2] <= (imp_->phase5_fd[2] + 2.0))
                    {
                        arm2_final_force[2] = imp_->phase5_fd[2];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase5_fd[i] + arm2_final_force[i] - imp_->phase5_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase5_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
           //Back
            else if(imp_->phase5 && !imp_->phase6)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_transform_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout()<<"Complete!: "
                           <<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                           <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;
                }

                forceUpperLimit(arm2_final_force, limit_area);
                forceDeadZone(arm2_final_force, imp_->p6_deadzone);

                if(count() <= imp_->back_count + 5000)
                {


                    //Impedence Controller
                    for (int i = 0; i < 2; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase6_fd[i] + arm2_final_force[i] - imp_->phase6_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase6_M[i];
                    }


                    for (int i = 0; i < 2; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }



                    arm2_current_pos[2] += 0.00001;
                    saMove(arm2_current_pos, model_a2, 1);
                }
                else
                {
                    mout()<<"Test Complete! "<< '\t' <<"Current X Angle: "<< '\t'<< imp_->x_ << '\t' <<"Current Y Angle: "<< '\t'<< imp_->y_ 
                    << '\t'<<"Current Point: "<< '\t'<< imp_->p_<< '\t'<<"Current m: "<< '\t'<< imp_->dx_<< '\t'<<"Current n: "<< '\t'<< imp_->dy_<<std::endl;
                    imp_->phase6 = true;
                    return 0;
                }
            }



        }


        return 200000 - count();
    }
    RivetSearch::RivetSearch(const std::string& name)
    {
        aris::core::fromXmlString(command(),
         "<Command name=\"r_search\">"
         "	<GroupParam>"
         "	<Param name=\"x_degree\" default=\"0\" abbreviation=\"x\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	<Param name=\"search_x\" default=\"0\" abbreviation=\"m\"/>"
         "	<Param name=\"search_y\" default=\"0\" abbreviation=\"n\"/>"
         "	<Param name=\"point\" default=\"0\" abbreviation=\"p\"/>"
         "	</GroupParam>"
         "</Command>");
    }
    RivetSearch::~RivetSearch() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(RivetSearch)

    
    struct RivetHoleDection::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;
        bool phase6 = false;

        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};
        double arm1_temp_force2[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };
        double arm2_temp_force[6]{0};
        double arm2_temp_force2[6]{0};
        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };
        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };
        //Desired Force of Each Phase
        double phase2_fd[6]{ 0 };
        double phase3_fd[6]{ 0,0,10,0,0,0 };
        double phase4_fd[6]{ 0,0,5,0,0,0 };
        double phase5_fd[6]{ 0,0,10,0,0,0 };
        double phase6_fd[6]{ 0,0,0,0,0,0 };
        //Current Vel
        double v_c[6]{ 0 };
        //Impedence Parameter
        double phase3_B[6]{ 1500,1500,45000,3,3,3 };
        double phase3_M[6]{ 100,100,400,2,2,2 };
        //Search
        double phase4_B[6]{ 2500,2500,30000,3,3,3 };
        double phase4_M[6]{ 100,100,100,2,2,2 };
        //Alter Parameter
        double phase4_B2[6]{2500,2500,8000,3,3,3};
        double phase4_M2[6]{ 100,100,100,2,2,2 };
        double phase5_B[6]{ 5000,5000,25000,0,0,0 };
        double phase5_M[6]{ 100,100,100,0,0,0 };
        double phase6_B[6]{ 3500,3500,2500,0,0,0  };
        double phase6_M[6]{ 100,100,100,0,0,0 };
        //Force Counter
        int allign_count = 0;
        int success_count = 0;
        //Counter for force comp
        int start_count = 0;
        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;
        double current_pos_checkek[6] = {0};
        //Switch Angle
        double x_;
        double y_;
        //Switch Point
        int p_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Parameters for Pos Recoginistion
        double a_y = -0.0567;
        double b_y = -0.9679;
        double c_y = 0.0010;

        double a_z = 0.9745;
        double b_z = -0.0535;
        double c_z = 0.0063;

        double hole_pos[6]{0};
        double hole_angle[6]{0};


        //Dead Zone
        double p4_deadzone[6]{1.0,1.0,0.5,0,0,0};
        double p5_deadzone[6]{1,1,0.1,0,0,0};
        double p6_deadzone[6]{3,3,3,0,0,0};

        //Search Parameter
		double px[31] = {0, 0.00025, 0.0005, 0.00075, 0.001, 0.00125, 0.0015, 0.00175, 0.002, 0.00225, 0.0025, 
        0.00275, 0.003, 0.00325, 0.0035, 0.00375, 0.004, 0.00425, 0.0045, 0.00475, 	
		0.005, 0.00525, 0.0055, 0.00575, 0.006, 0.00625, 0.0065, 0.00675, 0.007, 0.00725, 0.0075};

		double py[31] = {0, -0.0002097, 0.0004195, -0.0006293, 0.0008391, -0.0010488, 0.0012586, -0.001468, 0.001678, 
        -0.001888, 0.0020977, -0.002308, 0.002517, -0.0027271, 0.002937, -0.003147, 0.003356	
		-0.003566, 0.003776, -0.003986,	0.004195, -0.004405, 0.004615, 	-0.004815, 0.00450, -0.004146, 0.003742, -0.003269, 0.002693, -0.001920, 0};

		double force_direction[30]{0};

		
		double each_count[30] = {213, 389, 589, 795, 1002, 1211, 1419, 1628, 1837, 2047, 2256, 2466, 2675, 2885, 3094, 3304, 3514, 3723,	 
		3933, 4143, 4352, 4562, 4767, 4709, 4375, 3996, 3558, 3033, 2360, 1018};

        double pos_error = 0;

        //Counter
        int search_start_count = 0;
        int search_counter = 0;

        int total_search_count = 0;

        //Time Modify
        double avg_trans_x_force = 0;

        double temp_trans_x_force = 0;

        int trans_force_counter = 0;

        double theta;
        double search_pos[2]{0};

        double search_start_pos[6]{0};

        //Input Direction
        double dy_;
        double dx_;

        //Documented Depth
        double depth;
        double y_movement;
        double x_movement;

        //Back
        double back_count;

        //Side Movement Jammed Check
        double check_disp = 0;
        int jammed_count = 0;
        int jammed_check_count = 0;

        //Real theta
        double real_theta = 0;
        //Gen coords
        std::vector<std::array<double, 2>> gen_points;
        //Init Pos
        //double init_pos[2]{-0.622144, -0.206780};//10mm
        // double init_pos[2]{-0.622925, -0.102052};//12mm
        double init_pos[2]{-0.6223, -0.206695};//new
        // double init_pos[2]{-0.622394, -0.1018};//12mm new
    };
    auto RivetHoleDection::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        //user define
		double area = 7.5;
		double sepreate = 0.25;
		double theta = 40 * PI / 180.0;
		//TODO: Check fully devide
		int size = area / sepreate;
		std::vector<double> px(size + 1);
		std::vector<double> py(size + 1);
		std::vector<double> distance(size);
		double area_end_x = area * cos(theta);
		double area_end_y = area * sin(theta);
		for (size_t i = 0; i < px.size(); ++i) 
		{
			px[i] = (i) * 0.25;

			if (px[i] <= area_end_x) 
			{
				py[i] = std::tan(theta) * px[i];
			}
			else 
			{
				py[i] = std::sqrt(area * area - px[i] * px[i]);
			}

			if ((i) % 2 == 0) 
			{
				py[i] = -py[i];
			}


		}
		// imp_->each_time.resize(size);
		// imp_->each_theta.resize(size);
		// for (size_t i = 0; i < distance.size(); ++i)
		// {
		// 	distance[i] = std::sqrt((px[i+1] - px[i])* (px[i + 1] - px[i]) + (py[i + 1] - py[i])*(py[i + 1] - py[i]));
			
		// 	// rounded to 3 decima
		// 	double temp_val = distance[i] / 2.0;
		// 	temp_val = std::round(temp_val * 1000.0) / 1000.0;
			
		// 	imp_->each_time[i] = 1000.0 * temp_val + 50;

		// 	imp_->each_theta[i] = (atan2(py[i + 1] - py[i], px[i + 1] - px[i]) / PI) * 180.0;

		// 	//mout() << "time: " << imp_->each_time[i] << std::endl;

		// }		
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
        std::ifstream file("gen_coordinates_new_100.txt");
        if (!file.is_open()) {
            std::cerr << "无法打开文件 gen_coordinates.txt!" << std::endl;
        }
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;  // 跳过空行
            std::istringstream iss(line);
            double x, y;
            if (iss >> x >> y) {
                imp_->gen_points.push_back({x, y});
            } else {
                std::cerr << "解析错误：" << line << std::endl;
            }
        }
        file.close();

        // 2. 打印验证
        std::cout << "共读取 " << imp_->gen_points.size() << " 个坐标：" << std::endl;
    }
    auto RivetHoleDection::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);
        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));
        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};
        GravComp gc;
        SearchDir sd;
        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };
        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};
        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};
        double net_disp = 0;
        imp_->x_ = doubleParam("x_degree");
        imp_->y_ = doubleParam("y_degree");
        imp_->p_ = int32Param("point");
        imp_->dy_ = doubleParam("search_y");
        imp_->dx_ = doubleParam("search_x");
        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[0];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = -actual_force_[2];

                transform_force_[3] = -actual_force_[3];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = -actual_force_[5];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_, int num_)
        {

            bool isValid = true;
            for (int i = 0; i < num_; i++)
            {
                if (abs(current_force_[i]) > force_check_[i])
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 50 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() <<'\t'<<"Pos Error: "<<imp_->pos_error<< std::endl;
                    if (imp_->success_count >= count_)
                    {
                        imp_->success_count = 0;
                        imp_->allign_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        auto boltJammedCheck = [&](double net_disp_, int count_)
        {
            if(count()%500 == 0)
            {
                imp_->check_disp = net_disp_;
            }

            bool isValid = true;

            if (abs(imp_->check_disp - net_disp_) >= 0.0001)
            {
                isValid = false;
            }

            if(isValid)
            {
                if(imp_->jammed_count == 0)
                {
                    imp_->jammed_count = count();
                }

                if((count() - imp_->jammed_count) % 100 == 0)
                {
                    imp_->jammed_check_count++;
                    mout() << "Check " << imp_->jammed_check_count << '\t' << "Current Count: " << count() << '\t' << "d_disp: " << abs(imp_->check_disp - net_disp_)<< std::endl;
                    if(imp_->jammed_check_count >= count_)
                    {
                        imp_->jammed_count = 0;
                        imp_->jammed_check_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->jammed_check_count = 0;
            }

            return false;
        };

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }
        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);
        if(!imp_->init)
        {
            // double assem_pos[6]{ -0.622364, -0.206756, 0.08, PI, 0, PI };//10mm
            double assem_pos[6]{ -0.6223, -0.206695, 0.08, PI, 0, PI };//new
            // double assem_pos[6]{ -0.622925, -0.102052, 0.067, PI, 0, PI  }; //12mm
            // double assem_pos[6]{ -0.622394, -0.1018, 0.08, PI, 0, PI  }; //12mm new
            double init_angle[6]{0};
            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }
            model_a2.getInputPos(init_angle);
            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }
            saJointMove(init_angle, 1);
            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
            }
        }
        else
        {
            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);
            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);
            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);
            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);
            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }
            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);
            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);
            for (size_t i = 0; i < 6; i++)
            {
                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }
            imp_->depth = imp_->search_start_pos[2] - arm2_current_pos[2];
            imp_->y_movement = imp_->search_start_pos[1] - arm2_current_pos[1];
            imp_->x_movement = imp_->search_start_pos[0] - arm2_current_pos[0];
            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                // double assem_pos[6]{ -0.622144, -0.206780, 0.055, PI, 0, PI  }; //10mm
                double assem_pos[6]{ -0.6223, -0.206695, 0.055, PI, 0, PI  }; //10mm new
                // double assem_pos[6]{ -0.622925, -0.102052, 0.067, PI, 0, PI  }; //12mm
                // double assem_pos[6]{ -0.622394, -0.1018, 0.055, PI, 0, PI  }; //12mm new
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };
                //Define Initial Rotate displacment
                double rotate_angle[3]{ (imp_->x_ )* 2 * PI / 360, (imp_->y_)* 2 * PI / 360, 0 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };
                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");
               //Test Set
                // std::array<std::array<double, 3>, 15> points =
                // {
                //     {
                //         {0.00229, 0.00017, 0},
                //         {-0.00143, -0.00127, 0},
                //         {-0.00163, 0.00081, 0},
                //         {-0.00195, -0.00028, 0},
                //         {-0.0025, 0.00118, 0},
                //         {0.00142, -0.00075, 0},
                //         {-0.00074, -0.00059, 0},
                //         {-0.00209, 0.00189, 0},
                //         {-0.00143, -0.00127, 0},
                //         {-0.00114, -0.00262, 0},
                //         {0.00101, 0.0028, 0},
                //         {0.0009, 0.00078, 0},
                //         {0.00099, 0.00256, 0},
                //         {-0.0009, 0.00091, 0},
                //         {-0.00158, 0.0023, 0}
                //     }
                // };

                // //Train Set
                // std::array<std::array<double, 3>, 50> points =
                // {
                //     {

                //         {-0.003, 0, 0},
                //         {-0.002, 0, 0},
                //         {-0.0025, 0, 0},
                //         {0.0028, 0, 0},
                //         {0.002, 0, 0},
                //         {0.0025, 0, 0},
                //         {0, 0.0025, 0},
                //         {0, 0.002, 0},
                //         {0, 0.003, 0},
                //         {0, -0.0025, 0},
                //         {0, -0.002, 0},
                //         {0, -0.003, 0},    
                //         {-0.00212, -0.00212, 0},
                //         {-0.00141, -0.00141, 0},
                //         {-0.00075, -0.00075, 0},
                //         {-0.00212, 0.00212, 0},
                //         {-0.00141, 0.00141, 0},
                //         {-0.00175, 0.00175, 0},
                //         {0.00212, -0.00212, 0},
                //         {0.00141, -0.00141, 0},
                //         {0.00175, -0.00175, 0},
                //         {0.00212, 0.00212, 0},
                //         {0.00141, 0.00141, 0},
                //         {0.00175, 0.00175, 0},
                //         {-0.00225, 0.00261, 0},
                //         {0.00097, 0.0014, 0},
                //         {0.00244, 0.00105, 0},
                //         {-0.00099, 0.00261, 0},
                //         {0.00204, -0.00178, 0},
                //         {0.00178, -0.00078, 0},
                //         {0.00226, 0.00198, 0},
                //         {0.00124, 0.00275, 0},
                //         {0.00278, -0.00228, 0},
                //         {-0.00208, -0.00233, 0},
                //         {-0.00105, 0.00237, 0},
                //         {-0.00244, -0.00249, 0},
                //         {0.00238, 0.00256, 0},
                //         {-0.00242, 0.00093, 0},
                //         {-0.00097, -0.00142, 0},
                //         {-0.00217, -0.002, 0},
                //         {0.0025, 0.00241, 0},
                //         {0.00216, -0.00234, 0},
                //         {0.00103, -0.00129, 0},
                //         {0.00166, 0.00178, 0},
                //         {0.0017, -0.00144, 0},
                //         {0.00235, -0.00095, 0},
                //         {-0.00181, -0.00113, 0},
                //         {-0.00142, -0.00067, 0},
                //         {-0.00171, 0.00043, 0},
                //         {0.00078, -0.00182, 0}

                //     }
                // };
                if(imp_->p_ < 0 || imp_->p_ >= imp_->gen_points.size())
                {
                    mout()<<"Error Points Index"<<std::endl;
                    return 0;
                }
                for(size_t i = 0; i < 2; i++)
                {
                    assem_pos[i] += imp_->gen_points[imp_->p_][i];
                }
                eeA2.setP(assem_pos);
                if (model_a2.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }
                model_a2.getInputPos(assem_angle);
                saJointMove(assem_angle, 1);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {
                    imp_->phase1 = true;
                    imp_->start_count = count();
                    mout() << "Assembly Start !" << std::endl;
                    mout()<<"Current Point Index: "<<'\t'<<imp_->p_<<'\t'<<imp_->gen_points[imp_->p_][0]<<'\t'<<imp_->gen_points[imp_->p_][1]<<std::endl;
                    imp_->real_theta = atan2(-imp_->gen_points[imp_->p_][1], -imp_->gen_points[imp_->p_][0]);
                }
            }
            //Phase 2 Contact, Have Certain Position Adjustment
            else if (imp_->phase1 && !imp_->phase2)
            {
                if(count() == imp_->start_count + 2000)
                {
                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;
                }
                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };
                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);
                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;

                }
                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase2 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        master()->logFileRawName(std::string("/home/kaanh/Desktop/kaanhbin/10mm_new_test/stable_" + aris::core::logFileTimeFormat(std::chrono::system_clock::now())).c_str());
                        eeA2.getP(imp_->search_start_pos);
                        break;
                    }
                }
                if (!imp_->phase2)
                {
                    arm2_current_pos[2] -= 0.000002;
                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 3 Stable Contact
            else if (imp_->phase2 && !imp_->phase3)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;
                double init_error = 0;
                double x_error = 0;
                double y_error = 0;
                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }
                // if (count()%2 == 0)
                // {
                //    lout()<< arm2_final_force[0] <<'\t'<< arm2_final_force[1] <<'\t'<<arm2_final_force[2] 
                //    <<'\t'<<arm2_final_force[3]<<'\t'<<arm2_final_force[4]<<'\t'<<arm2_final_force[5]
                //    <<'\t'<<imp_->depth<<'\t'<< imp_->y_movement <<'\t'<< imp_->x_movement <<'\t'<< count() << std::endl;
                // }
                
                if (posCheck(arm2_current_pos, 5))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                    // eeA2.getP(imp_->search_start_pos);
                    sd.getDir(arm2_final_force, imp_->theta);
                    //imp_->theta = std::atan2(imp_->dy_, imp_->dx_);
                    mout()<<"Real theta: "<<imp_->real_theta*57.3<<'\t'<<"Cal theta: "<<imp_->theta*57.3<<std::endl;
                    for(int i = 0; i<6; i++){
                        imp_->v_c[i] = 0;
                    }
                    imp_->total_search_count = count();
                    //mout()<<imp_->theta * 57.3<<'\t'<<imp_->real_theta * 57.3<<std::endl;
                    x_error = std::abs(arm2_current_pos[0]-imp_->init_pos[0]);
                    y_error = std::abs(arm2_current_pos[1]-imp_->init_pos[1]);
                    init_error = sqrt(x_error*x_error + y_error*y_error);
                    lout()<<imp_->theta * 57.3<<'\t'<<imp_->real_theta * 57.3 <<'\t' << init_error << '\t' <<count()<<'\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 <<std::endl;
                    // imp_->back_count = count();
                }
                else
                {
                    if(arm2_final_force[2] >= (imp_->phase3_fd[2] - 1) && arm2_final_force[2] <= (imp_->phase3_fd[2] + 5.5)){
                        arm2_final_force[2] = imp_->phase3_fd[2];
                    }
                    //Impedence Controller
                    for (int i = 2; i < 3; i++)
                    {
                        //da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_final_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }
                    for (int i = 2; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];
                    }
                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 4 Fine Search
            else if (imp_->phase3 && !imp_->phase4)
            {
                double arm2_check_force[6]{0,0,0,0,0,0};
                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                imp_->pos_error = imp_->search_start_pos[0] - arm2_current_pos[0];
                net_disp = sqrt(imp_->x_movement*imp_->x_movement + imp_->y_movement*imp_->y_movement);
                if (count()%25 == 0)
                {
                   lout()<< arm2_final_force[0] <<'\t'<< arm2_final_force[1] <<'\t'<<arm2_final_force[2] 
                   <<'\t'<<arm2_final_force[3]<<'\t'<<arm2_final_force[4]<<'\t'<<arm2_final_force[5]
                   <<'\t'<<imp_->depth<<'\t'<< imp_->y_movement <<'\t'<< imp_->x_movement <<'\t'<< count() << std::endl;
                }
                //0.00015
                if (boltJammedCheck(net_disp, 8))
                {
                    imp_->phase4 = true;
                    mout() << "Pos 4 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    mout()<<"Total Search Time: "<< (count() - imp_->total_search_count) << std::endl;
                    //Delimiter​​
                    lout() << 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t'<< 0 << '\t' << 0 <<std::endl; 
                    //return 0;
                }
                else
                {
                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p4_deadzone);
                    if(arm2_final_force[2] >= (imp_->phase4_fd[2] - 0.01) && arm2_final_force[2] <= (imp_->phase4_fd[2] + 4.5))
                    {
                        arm2_final_force[2] = imp_->phase4_fd[2];
                    }
                    double real_xy_force[2]{0};
                    for(int i = 0; i < 2; i++)
                    {
                        real_xy_force[i] = arm2_final_force[i];
                    }
                    double expected_force[2]{-5.0,0};
                    eeA2.getP(arm2_current_pos);
                    if(imp_->search_counter < 30)
                    {
                        //Only xy movment
                        //int next_counter = round((1-(abs(imp_->avg_trans_x_force / 6.0))) * imp_->each_count[imp_->search_counter]);
                        //imp_->each_theta[imp_->search_counter] = (1 - abs(imp_->avg_trans_x_force / 20.0))*(imp_->each_theta[imp_->search_counter]) - imp_->theta;
                        imp_->force_direction[imp_->search_counter] = (1 - abs(imp_->avg_trans_x_force / 18.0))*std::atan2((imp_->py[imp_->search_counter+1] - imp_->py[imp_->search_counter]), (imp_->px[imp_->search_counter+1] - imp_->px[imp_->search_counter])) - imp_->theta;
                        if(imp_->search_start_count <= imp_->each_count[imp_->search_counter])
                        {

                            double trans_z[4] = {cos(imp_->force_direction[imp_->search_counter]), -sin(imp_->force_direction[imp_->search_counter]),
                                                sin(imp_->force_direction[imp_->search_counter]), cos(imp_->force_direction[imp_->search_counter])};

                            double inv_z[4] = {cos(imp_->force_direction[imp_->search_counter]), sin(imp_->force_direction[imp_->search_counter]),
                                                -sin(imp_->force_direction[imp_->search_counter]), cos(imp_->force_direction[imp_->search_counter])};

                            double acc[2]{ 0 };
                            double dx[2]{ 0 };
                            double dt = 0.001;

                            double trans_dx[2]{0};
                            double trans_xy_force[2]{0};

                            aris::dynamic::s_mm(2,1,2,trans_z,real_xy_force,trans_xy_force);

                            if(trans_xy_force[0] >= (expected_force[0] - 1.5) && trans_xy_force[0] <= (expected_force[0] + 1.5))
                            {
                                trans_xy_force[0] = expected_force[0];
                            }


                            if(imp_->search_start_count >= round(2.0 * imp_->each_count[imp_->search_counter] / 3.0) && imp_->search_start_count <= imp_->each_count[imp_->search_counter])
                            {
                                imp_->trans_force_counter ++;
                                imp_->temp_trans_x_force += trans_xy_force[0];
                            }
                            if(imp_->search_start_count == imp_->each_count[imp_->search_counter])
                            {
                                imp_->avg_trans_x_force = imp_->temp_trans_x_force / imp_->trans_force_counter;
                            }

                            //Impedence Controller for X Y Movement
                            for (int i = 0; i < 2; i++)
                            {
                                // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                                acc[i] = (-expected_force[i] + trans_xy_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                            }


                            for (int i = 0; i < 2; i++)
                            {
                                imp_->v_c[i] += acc[i] * dt;
                                velDeadZone(imp_->v_c[i], max_vel[i]);
                                dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;

                            }

                            aris::dynamic::s_mm(2,1,2,inv_z,dx,trans_dx);

                            double acc_x = 0;
                            double dx_x = 0;

                            //Impedence Controller for Z Only
                            if(arm2_final_force[2] <= 1.5)
                            {
                                acc_x = (-imp_->phase4_fd[2] + arm2_final_force[2] - imp_->phase4_B2[2] * (imp_->v_c[2] - imp_->v_d[2])) / imp_->phase4_M2[2];
                                imp_->v_c[2] += acc_x * dt;
                                velDeadZone(imp_->v_c[2], max_vel[2]);
                                dx_x = imp_->v_c[2] * dt + acc_x * dt * dt;
                            }
                            else
                            {
                                acc_x = (-imp_->phase4_fd[2] + arm2_final_force[2] - imp_->phase4_B[2] * (imp_->v_c[2] - imp_->v_d[2])) / imp_->phase4_M[2];
                                imp_->v_c[2] += acc_x * dt;
                                velDeadZone(imp_->v_c[2], max_vel[2]);
                                dx_x = imp_->v_c[2] * dt + acc_x * dt * dt;
                            }


                            arm2_current_pos[2] += dx_x;

                            arm2_current_pos[0] += trans_dx[0];
                            arm2_current_pos[1] += trans_dx[1];

                            imp_->search_start_count++;


                            if(count() % 100 == 0)
                            {
    //                                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<
    //                                '\t'<<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t'
    //                                     <<"Trans_force: "<<trans_yz_force[0]<<'\t'<<trans_yz_force[1]<<std::endl;


//                                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<'\t'
//                                     <<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t'
//                                    << "Trans_force:"<<'\t'<<trans_yz_force[0]<<'\t'<<trans_yz_force[1] << '\t'
//                                    << "Counter:"<<'\t'<<imp_->search_counter<<'\t'
//                                    << "Deg:"<<'\t'<< 57.296 * imp_->force_direction[imp_->search_counter] <<std::endl;


                                mout()<<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t' <<std::endl;
                            }


                        }
                        else
                        {

                            for(int i = 0; i < 2; i++)
                            {
                                imp_->v_c[i] = 0;
                            }
                            imp_->search_counter++;
                            imp_->search_start_count = 0;
                            mout()<<"Search Counter: "<<imp_->search_counter<<std::endl;
                        }


                        saMove(arm2_current_pos, model_a2, 1);

                    }
                    else
                    {
                        mout()<<"Finish"<<std::endl;
                        return 0;
                    }
                }

            }
            //Phase 5 Maintain 2 Points Contact
            else if (imp_->phase4 && !imp_->phase5)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;
                double x_error = 0;
                double y_error = 0;
                double d_error = 0;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }
                if (count()%25 == 0)
                {
                   lout()<< arm2_final_force[0] <<'\t'<< arm2_final_force[1] <<'\t'<<arm2_final_force[2] 
                   <<'\t'<<arm2_final_force[3]<<'\t'<<arm2_final_force[4]<<'\t'<<arm2_final_force[5]
                   <<'\t'<<imp_->depth<<'\t'<< imp_->y_movement <<'\t'<< imp_->x_movement <<'\t'<< count() << std::endl;
                }
                if (posCheck(arm2_current_pos, 5))
                {
                    imp_->phase5 = true;
                    mout() << "Pos 5 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                    x_error = std::abs(imp_->init_pos[0]-arm2_current_pos[0]);
                    y_error = std::abs(imp_->init_pos[1]-arm2_current_pos[1]);
                    d_error = sqrt(x_error*x_error + y_error*y_error);
                    lout() << d_error << '\t' <<count()<<0<<'\t'<<0<<'\t'<<0<<'\t'<<0<<'\t'<<0<<'\t'<<0<<'\t'<<0<<'\t'<<0<<'\t'<<0<<std::endl; 
                    imp_->back_count = count();
                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p5_deadzone);

                    if(arm2_final_force[2] >= (imp_->phase5_fd[2] - 1) && arm2_final_force[2] <= (imp_->phase5_fd[2] + 1.0))
                    {
                        arm2_final_force[2] = imp_->phase5_fd[2];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase5_fd[i] + arm2_final_force[i] - imp_->phase5_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase5_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Back
            else if(imp_->phase5 && !imp_->phase6)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_transform_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout()<<"Complete!: "
                           <<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                           <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;
                }

                forceUpperLimit(arm2_final_force, limit_area);
                forceDeadZone(arm2_final_force, imp_->p6_deadzone);

                if(count() <= imp_->back_count + 3000)
                {


                    //Impedence Controller
                    for (int i = 0; i < 2; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase6_fd[i] + arm2_final_force[i] - imp_->phase6_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase6_M[i];
                    }


                    for (int i = 0; i < 2; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }



                    arm2_current_pos[2] += 0.000015;
                    saMove(arm2_current_pos, model_a2, 1);
                }
                else
                {
                    mout()<<"Test Complete! "<< '\t' <<"Current X Angle: "<< '\t'<< imp_->x_ << '\t' <<"Current Y Angle: "<< '\t'<< imp_->y_ 
                    << '\t'<<"Current Point: "<< '\t'<< imp_->p_<< '\t'<<"Current m: "<< '\t'<< imp_->dx_<< '\t'<<"Current n: "<< '\t'<< imp_->dy_<<std::endl;
                    imp_->phase6 = true;
                    return 0;
                }
            }
        }
        return 200000 - count();
    }
    RivetHoleDection::RivetHoleDection(const std::string& name)
    {
        aris::core::fromXmlString(command(),
         "<Command name=\"r_hd\">"
         "	<GroupParam>"
         "	<Param name=\"x_degree\" default=\"0\" abbreviation=\"x\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	<Param name=\"search_x\" default=\"0\" abbreviation=\"m\"/>"
         "	<Param name=\"search_y\" default=\"0\" abbreviation=\"n\"/>"
         "	<Param name=\"point\" default=\"0\" abbreviation=\"p\"/>"
         "	</GroupParam>"
         "</Command>");
    }
    RivetHoleDection::~RivetHoleDection() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(RivetHoleDection)
    


    
    struct RivetCalib::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        //Force Compensation Parameter
        double comp_f[6]{ 0 };
        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };
        double arm1_temp_force[6]{0};
        double arm1_temp_force2[6]{0};
        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };
        double arm2_temp_force[6]{0};
        double arm2_temp_force2[6]{0};
        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };
        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };
        //Desired Force of Each Phase
        double phase3_fd[6]{ 0,0,6,0,0,0 };
        double phase4_fd[6]{ 0,0,0,0,0,0 };
        //Current Vel
        double v_c[6]{ 0 };
        //Impedence Parameter
        double phase3_B[6]{ 3500,3500,30000,0,0,0 };
        double phase3_M[6]{ 100,100,400,0,0,0 };
        double phase4_B[6]{ 3500,3500,2500,0,0,0  };
        double phase4_M[6]{ 100,100,100,0,0,0 };
        //Counter for force comp
        int start_count = 0;
        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;
        double current_pos_checkek[6] = {0};
        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };
        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };
        //Dead Zone
        double p3_deadzone[6]{1.0,1.0,0.5,0,0,0};
        double p4_deadzone[6]{3,3,3,0,0,0};
        //Back
        double back_count;
    };
    auto RivetCalib::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        GravComp gc;		
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto RivetCalib::executeRT() -> int
    {
        //dual transform modelbase into multimodel
        auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
        //at(0) -> Arm1 -> white
        auto& arm1 = dualArm.subModels().at(0);
        //at(1) -> Arm2 -> blue
        auto& arm2 = dualArm.subModels().at(1);
        //transform to model
        auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
        auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
        //End Effector
        auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
        auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));
        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };
        static double limit_area[6]{15,15,15,0.6,0.6,0.6};
        GravComp gc;
        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };
        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };
        double arm1_final_force[6]{0};
        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };
        double arm2_final_force[6]{0};

        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                throw std::runtime_error("Inverse Kinematics Position Failed!");
            }


            double x_joint[6]{ 0 };

            model_.getInputPos(x_joint);

            if (type_ == 0)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i].setTargetPos(x_joint[i]);
                }
            }
            else if (type_ == 1)
            {
                for (std::size_t i = 0; i < 6; ++i)
                {
                    controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                }
            }
            else
            {
                throw std::runtime_error("Arm Type Error");
            }
        };

        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[0];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = -actual_force_[2];

                transform_force_[3] = -actual_force_[3];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = -actual_force_[5];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }
        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);
        if(!imp_->init)
        {
            double assem_pos[6]{ -0.622759, -0.102215, 0.08, PI, 0, PI }; //12mm
            // double assem_pos[6]{ -0.622144, -0.206780, 0.08, PI, 0, PI  }; //10mm

            double init_angle[6]{0};
            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }
            model_a2.getInputPos(init_angle);
            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }
            saJointMove(init_angle, 1);
            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);
                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
                mout()<<"Init Complete"<<std::endl;
                imp_->init = true;
            }
        }
        else
        {
            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);
            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);
            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);
            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);
            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }
            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);
            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);
            for (size_t i = 0; i < 6; i++)
            {
                arm1_final_force[i] = arm1_transform_force[i] - imp_->arm1_start_force[i];
                arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
            }
            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                //double assem_pos[6]{ -0.622759, -0.102215, 0.070, PI, 0, PI  }; //12mm
                //double assem_pos[6]{ -0.622144, -0.206780, 0.055, PI, 0, PI  }; //10mm
                double assem_pos[6]{ -0.622759, -0.102215, 0.055, PI, 0, PI  }; //12mm new
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };
                eeA2.setP(assem_pos);
                if (model_a2.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }
                model_a2.getInputPos(assem_angle);
                saJointMove(assem_angle, 1);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {
                    imp_->phase1 = true;
                    imp_->start_count = count();
                    mout() << "Calib Start !" << std::endl;
                }
            }
            //Phase 2 Contact, Have Certain Position Adjustment
            else if (imp_->phase1 && !imp_->phase2)
            {
                if(count() == imp_->start_count + 2000)
                {
                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;
                }
                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };
                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);
                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;

                }
                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase2 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        break;
                    }
                }
                if (!imp_->phase2)
                {
                    arm2_current_pos[2] -= 0.000002;
                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 3 Stable Contact
            else if (imp_->phase2 && !imp_->phase3)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;
                double init_error = 0;
                double x_error = 0;
                double y_error = 0;
                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }
                if (posCheck(arm2_current_pos, 5))
                {
                    imp_->phase3 = true;
                    imp_->back_count = count();
                    mout() << "Pos 3 Complete" << std::endl;
                    mout() << "Calib Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                }
                else
                {
                    if(arm2_final_force[2] >= (imp_->phase3_fd[2] - 2.0) && arm2_final_force[2] <= (imp_->phase3_fd[2] + 3.0)){
                        arm2_final_force[2] = imp_->phase3_fd[2];
                    }
                    forceDeadZone(arm2_final_force, imp_->p3_deadzone);
                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        //da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_final_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }
                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];
                    }
                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Back
            else if(imp_->phase3 && !imp_->phase4)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };
                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;
                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_transform_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout()<<"Complete!: "
                           <<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                           <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;
                }
                forceUpperLimit(arm2_final_force, limit_area);
                forceDeadZone(arm2_final_force, imp_->p4_deadzone);
                if(count() <= imp_->back_count + 3000)
                {
                    //Impedence Controller
                    for (int i = 0; i < 2; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + arm2_final_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }
                    for (int i = 0; i < 2; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];
                    }
                    arm2_current_pos[2] += 0.000015;
                    saMove(arm2_current_pos, model_a2, 1);
                }
                else
                {
                    mout()<<"Calib Complete! "<<std::endl;
                    imp_->phase4 = true;
                    return 0;
                }
            }
        }
        return 200000 - count();
    }
    RivetCalib::RivetCalib(const std::string& name)
    {
        aris::core::fromXmlString(command(),
                    "<Command name=\"r_calib\"/>");
    }
    RivetCalib::~RivetCalib() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(RivetCalib)
    
    ARIS_REGISTRATION {
        aris::core::class_<RivetInit>("RivetInit")
            .inherit<aris::plan::Plan>();
        aris::core::class_<RivetStart>("RivetStart")
            .inherit<aris::plan::Plan>();
        aris::core::class_<RivetOut>("RivetOut")
            .inherit<aris::plan::Plan>();
        aris::core::class_<RivetSearch>("RivetSearch")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Arm2Init>("Arm2Init")
            .inherit<aris::plan::Plan>();
        aris::core::class_<RivetHoleDection>("RivetHoleDection")
            .inherit<aris::plan::Plan>();
        aris::core::class_<RivetCalib>("RivetCalib")
            .inherit<aris::plan::Plan>();
    }

}