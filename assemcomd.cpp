#include "assemcomd.hpp"
#include "robot.hpp"
#include "gravcomp.hpp"
#include <array>

using namespace std;

namespace assemble
{

    struct HoleInPeg::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        // ALter Pos, Contact, Hold Pos, Get 1s Data
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        // Rough Search to Point, Get Data
        bool phase5 = false;
        bool phase6 = false;
        bool phase7 = false;
        bool phase8 = false;
        // Fine Search
        bool phase9 = false;

        // //Hole in Peg// //
        // Two Points Contact
        bool phase10 = false;
        // Hole Allign
        bool phase11 = false;
        // Maintain Contact
        bool phase12 = false;
        // Plate Allign
        bool phase13 = false;

        // //Final Allign// //
        // Plate Out 5mm Proxi
        bool phase14 = false;
        // Peg in Fine Allign
        bool phase15 = false;
        // Plate Contact
        bool phase16 = false;

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
        double phase3_fd[6]{ 5,0,0,0,0,0 };
        double phase7_fd[6]{ 5,0,0,0,0,0 };
        double phase9_fd[6]{ 5,0,0,0,0,0 };
        double phase10_fd[6]{ 5,0,0,0,0,0 };
        double phase11_fd[6]{ 3.5,0,0,0,0,0 };
        double phase12_fd[6]{ 3.5,0,0,0,0,0 };
        double phase13_fd[6]{ 7.5,0,0,0,0,0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 25000,1500,1500,3,3,3 };
        double phase3_M[6]{ 2000,100,100,2,2,2 };

        double phase7_B[6]{ 25000,1500,1500,3,3,3 };
        double phase7_M[6]{ 2000,100,100,2,2,2 };

        double phase9_B[6]{ 5000,2500,2500,3,3,3 };   //fine-search
        double phase9_M[6]{ 100,100,100,2,2,2 };

        double phase10_B[6]{ 3000,2000,2000,0,0,0 };   //2-points
        double phase10_M[6]{ 200,100,100,0,0,0 };

        double phase11_B[6]{ 3000,3500,3500,4.5,3.5,3.5 }; //HIP
        double phase11_M[6]{ 150,100,100,2.5,1.0,1.0 };

        double phase12_B[6]{ 3000,3500,3500,0,0,0 }; //move till contact
        double phase12_M[6]{ 100,100,100,0,0,0 };

        double phase13_B[6]{ 3500,3500,3500,15,80,80 }; // plate allign
        double phase13_M[6]{ 100,100,100,10,5,5 };


        //Counter
        int current_count = 0;
        int allign_count = 0;
        int success_count = 0;


        int start_count = 0;
        int complete_count = 0;
        int back_count = 0;

        int accumulation_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        double current_pos_checkek[6] = {0};

        //Switch Angle
        double z_;
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
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;

        double a_z = 0.9751;
        double b_z = -0.0553;
        double c_z = 0.0020;

        double dy = 0;
        double dz = 0;

        double hole_pos[6]{0};
        double hole_angle[6]{0};

        //Parameters for Fine Search
        //Dead Zone
        double p9_deadzone[6]{1.0,1.0,1.0,0,0,0};
        double p10_deadzone[6]{0.1,3,3,0,0,0};
        double p11_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p12_deadzone[6]{0.1,3,3,0,0,0};
        double p13_deadzone[6]{0.1,5,5,0.3,0.3,0.3};

        //Search Parameter
        double px[21] = {0,	0.00025, 0.00050, 0.00075, 0.001, 0.00125, 0.00150, 0.00175, 0.002,
        0.00225, 0.0025, 0.00275, 0.003, 0.00325, 0.0035, 0.00375, 0.004, 0.00425, 0.0045, 0.00475, 0.005,};

        double py[21] = {0, -0.0001443, 0.0002887, -0.000433, 0.0005774, -0.0007217, 0.000866, -0.0010104, 0.0011547, -0.001299, 0.0014434, -0.0019485,
        0.0024536, -0.0029587, 0.0034637, -0.0033072, 0.003, -0.0026339, 0.0021794, -0.0015612, 0};

        double force_direction[20]{0};
        double each_count[20] = {194, 300, 432, 570, 711, 854, 996, 1140, 1283, 1427, 1751, 2255, 2759, 3264, 3438, 3206, 2870, 2460, 1925, 841};
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

        //Stroaged Pos Data
        double tpcontact_pos[6]{0};




    };
    auto HoleInPeg::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;

    }
    auto HoleInPeg::executeRT() -> int
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

        double mz_comp;

        imp_->z_ = doubleParam("z_degree");

        imp_->y_ = doubleParam("y_degree");

        imp_->p_ = int32Param("point");


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
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
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

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {
            double assem_pos[6]{ -0.690, 0.012466, 0.291196, PI / 4, -PI / 2, - PI / 4 };
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


            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ -0.690, 0.012466, 0.291196, PI / 4, -PI / 2, - PI / 4 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate displacment

                double rotate_angle[3]{ 0, (imp_->y_)* 2 * PI / 360, (imp_->z_ )* 2 * PI / 360 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");


                //Define Initial Position displacment
//                std::array<std::array<double, 3>, 13> points =
//                {
//                    {
//                        {0, -0.007, 0},
//                        {0, -0.009, 0},
//                        {0, -0.011, 0},
//                        {0, -0.013, 0},
//                        {0, -0.015, 0},
//                        {0, -0.017, 0},
//                        {0, -0.020, 0},
//                        {0, 0.007, 0},
//                        {0, 0.009, 0},
//                        {0, 0.011, 0},
//                        {0, 0.013, 0},
//                        {0, 0.015, 0},
//                        {0, 0.017, 0},
//                    }
//                };


//                std::array<std::array<double, 3>, 14> points =
//                {
//                    {
//                        {0, 0, -0.007},
//                        {0, 0, -0.009},
//                        {0, 0, -0.011},
//                        {0, 0, -0.013},
//                        {0, 0, -0.015},
//                        {0, 0, -0.017},
//                        {0, 0, -0.020},
//                        {0, 0, 0.007},
//                        {0, 0, 0.009},
//                        {0, 0, 0.011},
//                        {0, 0, 0.013},
//                        {0, 0, 0.015},
//                        {0, 0, 0.017},
//                        {0, 0, 0.020},
//                    }
//                };



//                std::array<std::array<double, 3>, 10> points =
//                {
//                    {
//                        {0, -0.008, -0.008},
//                        {0, -0.012, -0.010},
//                        {0, -0.015, -0.014},
//                        {0, 0.012, -0.013},
//                        {0, 0.013, 0.015},
//                        {0, 0.006, -0.017},
//                        {0, -0.006, -0.020},
//                        {0, 0.002, 0.007},
//                        {0, -0.003, 0.009},
//                        {0, 0.005, 0.011},
//                    }
//                };


                std::array<std::array<double, 3>, 10> points =
                {
                    {
                        {0, -0.005, 0.015},
                        {0, 0.005, 0.015},
                        {0, -0.010, -0.014},
                        {0, 0.012, -0.013},
                        {0, 0.009, 0.015},

                        {0, -0.007, 0.008},
                        {0, -0.020, 0.020},
                        {0, -0.005, -0.008},
                        {0, 0.008, 0.007},
                        {0, -0.013, -0.006},
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
                    arm2_current_pos[0] -= 0.000003;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Hold Pos To Get 1s Data
            else if (imp_->phase2 && !imp_->phase3)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0,0,0,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 18.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 8))
                {
                    imp_->phase3 = true;
                    mout() << "Pos Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();

                }
                else
                {

                    if(arm2_final_force[0] >= (imp_->phase3_fd[0] - 0.01) && arm2_final_force[0] <= (imp_->phase3_fd[0] + 2))
                    {
                        arm2_final_force[0] = imp_->phase3_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_final_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 4 Get 1s Data to Rough Search
            else if (imp_->phase3 && !imp_->phase4)
            {
                if(count() <= imp_->complete_count + 1050)
                {
                    if(count() % 50 == 0 && imp_->accumulation_count < 20)
                    {
                        mout()<<"A1_Force"<<"\t"<<arm1_final_force[0]<<"\t"<<arm1_final_force[1]<<"\t"<<arm1_final_force[2]<<"\t"
                                <<arm1_final_force[3]<<"\t"<<arm1_final_force[4]<<"\t"<<arm1_final_force[5]<<"\t"<<"\t"<<"\t"<<"\t"
                               <<"A2_Force"<<"\t"<< arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                              << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;


                        for(int i = 0; i<6; i++)
                        {
                            imp_->arm1_temp_force[i] += arm1_final_force[i];
                            imp_->arm2_temp_force[i] += arm2_final_force[i];

                        }

                          imp_->accumulation_count ++;

                        //mout()<<"count: "<<imp_->accumulation_count<<std::endl;

                    }
                }
                else
                {
                    double arm1_avg_force[6]{0};
                    double arm2_avg_force[6]{0};

                    for(int i = 0; i<6; i++)
                    {
                        arm1_avg_force[i] = imp_->arm1_temp_force[i] / 20.0;
                        arm2_avg_force[i] = imp_->arm2_temp_force[i] / 20.0;
                    }


                    imp_-> dy = -0.0825 + ((imp_->a_y * arm2_avg_force[1] - arm2_avg_force[5] + imp_->c_y) / (imp_->b_y * arm2_avg_force[0]));
                    imp_-> dz = 0.0 - ((arm2_avg_force[4] - imp_->c_z + imp_->b_z * arm2_avg_force[2]) / (imp_->a_z * arm2_avg_force[0]));




                    mout()<<"Data Acquired! Current Point: "<< imp_->p_ << '\t' <<"Current Z Angle: "<< imp_->z_ << '\t' <<"Current Y Angle: "<< imp_->y_ <<std::endl;

                    mout()<<"A1_Force"<<"\t"<<arm1_avg_force[0]<<"\t"<<arm1_avg_force[1]<<"\t"<<arm1_avg_force[2]<<"\t"
                            <<arm1_avg_force[3]<<"\t"<<arm1_avg_force[4]<<"\t"<<arm1_avg_force[5]<<"\t"<<"\t"<<"\t"<<"\t"
                           <<"A2_Force"<<"\t"<<arm2_avg_force[0]<<"\t"<<arm2_avg_force[1]<<"\t"<<arm2_avg_force[2]<<"\t"
                           <<arm2_avg_force[3]<<"\t"<<arm2_avg_force[4]<<"\t"<<arm2_avg_force[5]<<std::endl;

                     mout()<<"Calculated Pos: " << imp_->dy << '\t' <<imp_->dz << std::endl;

                    imp_->back_count = count();
                    imp_->accumulation_count = 0;
                    imp_->phase4 = true;
                }

            }
            //Phase 5 Back & Move to Hole
            else if (imp_->phase4 && !imp_->phase5)
            {

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 10.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout()<<"Curret Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t'<< arm2_current_pos[2] << '\t'
                         << arm2_current_pos[3] << '\t'<< arm2_current_pos[4] << '\t'<< arm2_current_pos[5] << '\t'<< std::endl;
                }


                if(count() <= imp_->back_count + 2500)
                {
                    arm2_current_pos[0] += 0.00001;
                    saMove(arm2_current_pos, model_a2, 1);

                }
                else if (count() == imp_->back_count + 2501)
                {
                    eeA2.getP(imp_->hole_pos);
                    imp_->hole_pos[1] -= imp_->dy;
                    imp_->hole_pos[2] -= imp_->dz;

                    mout()<<"Hole Pos: " << imp_->hole_pos[0] << '\t' << imp_->hole_pos[1] << '\t'<< imp_->hole_pos[2] << '\t'
                         << imp_->hole_pos[3] << '\t'<< imp_->hole_pos[4] << '\t'<< imp_->hole_pos[5] << '\t'<< std::endl;

                    eeA2.setP(imp_->hole_pos);
                    if(model_a2.inverseKinematics())
                    {
                        mout()<<"Inverse Failed"<<std::endl;
                        return 0;
                    }
                    model_a2.getInputPos(imp_->hole_angle);

                }
                else if(count() > imp_->back_count + 2501)
                {

                    saJointMove(imp_->hole_angle, 1);
                    if (motorsPositionCheck(current_sa_angle, imp_->hole_angle, 6))
                    {

                        imp_->phase5 = true;
                        imp_->start_count = count();
                        mout()<<"Current Pos: "<< arm2_current_pos[0] <<'\t'<< arm2_current_pos[1] <<'\t' << arm2_current_pos[2]<<std::endl;
                        mout() << "Pos Complete !" << std::endl;
                    }
                }
            }
            //Phase 6 Contact
            else if (imp_->phase5 && !imp_->phase6)
            {
                if(count() == imp_->start_count + 500)
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


                //Arm2
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
                        imp_->phase6 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;



                        break;

                    }

                }
                if (!imp_->phase6)
                {
                    arm2_current_pos[0] -= 0.000003;
                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 7 Hold Pos
            else if (imp_->phase6 && !imp_->phase7)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0,0,0,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 18.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 8))
                {
                    imp_->phase7 = true;

                    mout() << "Pos Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();

                }
                else
                {

                    if(arm2_final_force[0] >= (imp_->phase7_fd[0] - 0.01) && arm2_final_force[0] <= (imp_->phase7_fd[0] + 2))
                    {
                        arm2_final_force[0] = imp_->phase7_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase7_fd[i] + arm2_final_force[i] - imp_->phase7_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase7_M[i];
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 8 Get 1s Data
            else if (imp_->phase7 && !imp_->phase8)
            {
                if(count() <= imp_->complete_count + 1050)
                {
                    if(count() % 50 == 0 && imp_->accumulation_count < 20)
                    {
                        mout()<<"A1_Force"<<"\t"<<arm1_final_force[0]<<"\t"<<arm1_final_force[1]<<"\t"<<arm1_final_force[2]<<"\t"
                                <<arm1_final_force[3]<<"\t"<<arm1_final_force[4]<<"\t"<<arm1_final_force[5]<<"\t"<<"\t"<<"\t"<<"\t"
                               <<"A2_Force"<<"\t"<< arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                              << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;


                        for(int i = 0; i<6; i++)
                        {
                            imp_->arm1_temp_force2[i] += arm1_final_force[i];
                            imp_->arm2_temp_force2[i] += arm2_final_force[i];

                        }

                          imp_->accumulation_count ++;

                        //mout()<<"count: "<<imp_->accumulation_count<<std::endl;

                    }
                }
                else
                {
                    double arm1_avg_force[6]{0};
                    double arm2_avg_force[6]{0};

                    for(int i = 0; i<6; i++)
                    {
                        arm1_avg_force[i] = imp_->arm1_temp_force2[i] / 20.0;
                        arm2_avg_force[i] = imp_->arm2_temp_force2[i] / 20.0;
                    }

                    double dz = 0;
                    double dy = 0;

                    double real_dy = 0;
                    double real_dz = 0;

                    real_dy = arm2_current_pos[1] - 0.012466;
                    real_dz = arm2_current_pos[2] - 0.291196;

                    dy = -0.0825 + ((imp_->a_y * arm2_avg_force[1] - arm2_avg_force[5] + imp_->c_y) / (imp_->b_y * arm2_avg_force[0]));
                    dz = 0.0 - ((arm2_avg_force[4] - imp_->c_z + imp_->b_z * arm2_avg_force[2]) / (imp_->a_z * arm2_avg_force[0]));


                    if(dy <= 0.00001)
                    {
                        dy = 0;
                    }
                    if(dz <= 0.00001)
                    {
                        dz = 0;
                    }

                    //Fine Search
                    eeA2.getP(imp_->search_start_pos);
                    imp_->theta = std::atan2(-dz, -dy);

                    if(imp_->theta == 0)
                    {
                        mout()<<"Error Search Angle"<<std::endl;
                        return 0;
                    }

                    for(int i = 0; i < 20; i++)
                    {
                        imp_->force_direction[i] = std::atan2((imp_->py[i+1] - imp_->py[i]), (imp_->px[i+1] - imp_->px[i])) - imp_->theta;
                    }

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    master()->logFileRawName("search_contact_full");

                    imp_->total_search_count = count();


                    mout()<<"Data Acquired! Current Point: "<< imp_->p_ << '\t' <<"Current Z Angle: "<< imp_->z_ << '\t' <<"Current Y Angle: "<< imp_->y_ <<std::endl;

                    mout()<<"A2_Force"<<"\t"<<arm2_avg_force[0]<<"\t"<<arm2_avg_force[1]<<"\t"<<arm2_avg_force[2]
                            <<'\t'<<'\t'<<"current pos"<<'\t'<< arm2_current_pos[1] << '\t' << arm2_current_pos[2]
                            <<'\t'<<'\t'<<"d pos"<<'\t'<< dy << '\t' <<dz
                            <<'\t'<<'\t'<<"real d pos"<<'\t'<< real_dy << '\t' << real_dz <<std::endl;

                    imp_->accumulation_count = 0;
                    imp_->phase8 = true;
                }
            }
            //Phase 9 Fine Search
            else if (imp_->phase8 && !imp_->phase9)
            {
                double arm2_check_force[6]{0.7,0,0,0,0,0};

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

                if (forceCheck(arm2_final_force, arm2_check_force, 6, 1) && abs(imp_->search_start_pos[0] - arm2_current_pos[0]) >= 0.00015)
                {
                    imp_->phase9 = true;
                    mout() << "Pos 9 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    mout()<<"Total Search Time: "<< (count() - imp_->total_search_count) << std::endl;

                    //return 0;

                }
                else
                {

                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p9_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase9_fd[0] - 0.01) && arm2_final_force[0] <= (imp_->phase9_fd[0] + 2.5))
                    {
                        arm2_final_force[0] = imp_->phase9_fd[0];
                    }


                    double real_yz_force[2]{0};

                    for(int i = 0; i < 2; i++)
                    {
                        real_yz_force[i] = arm2_transform_force[i+1];
                    }

                    double expected_force[2]{-5.0,0};

                    eeA2.getP(arm2_current_pos);


                    if(imp_->search_counter < 20)
                    {
                        //Only yz movment

                        //int next_counter = round((1-(abs(imp_->avg_trans_x_force / 6.0))) * imp_->each_count[imp_->search_counter]);


                        imp_->force_direction[imp_->search_counter] = (1 - abs(imp_->avg_trans_x_force / 12.0))*std::atan2((imp_->py[imp_->search_counter+1] - imp_->py[imp_->search_counter]), (imp_->px[imp_->search_counter+1] - imp_->px[imp_->search_counter])) - imp_->theta;


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
                            double trans_yz_force[2]{0};

                            aris::dynamic::s_mm(2,1,2,trans_z,real_yz_force,trans_yz_force);

                            if(trans_yz_force[0] >= (expected_force[0] - 1.5) && trans_yz_force[0] <= (expected_force[0] + 1))
                            {
                                trans_yz_force[0] = expected_force[0];
                            }


                            if(imp_->search_start_count >= round(2.0 * imp_->each_count[imp_->search_counter] / 3.0) && imp_->search_start_count <= imp_->each_count[imp_->search_counter])
                            {
                                imp_->trans_force_counter ++;
                                imp_->temp_trans_x_force += trans_yz_force[0];
                            }
                            if(imp_->search_start_count == imp_->each_count[imp_->search_counter])
                            {
                                imp_->avg_trans_x_force = imp_->temp_trans_x_force / imp_->trans_force_counter;
                            }

                            //Impedence Controller
                            for (int i = 0; i < 2; i++)
                            {
                                // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                                acc[i] = (-expected_force[i] + trans_yz_force[i] - imp_->phase9_B[i+1] * (imp_->v_c[i+1] - imp_->v_d[i+1])) / imp_->phase9_M[i+1];
                            }


                            for (int i = 0; i < 2; i++)
                            {
                                imp_->v_c[i+1] += acc[i] * dt;
                                velDeadZone(imp_->v_c[i+1], max_vel[i+1]);
                                dx[i] = imp_->v_c[i+1] * dt + acc[i] * dt * dt;

                            }

                            aris::dynamic::s_mm(2,1,2,inv_z,dx,trans_dx);

                            double acc_x = 0;
                            double dx_x = 0;

                            acc_x = (-imp_->phase9_fd[0] + arm2_transform_force[0] - imp_->phase9_B[0] * (imp_->v_c[0] - imp_->v_d[0])) / imp_->phase9_M[0];
                            imp_->v_c[0] += acc_x * dt;
                            velDeadZone(imp_->v_c[0], max_vel[0]);
                            dx_x = imp_->v_c[0] * dt + acc_x * dt * dt;

                            arm2_current_pos[0] += dx_x;

                            arm2_current_pos[1] += trans_dx[0];
                            arm2_current_pos[2] += trans_dx[1];

                            imp_->search_start_count++;


                            if(count() % 100 == 0)
                            {
//                                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<
//                                '\t'<<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t'
//                                     <<"Trans_force: "<<trans_yz_force[0]<<'\t'<<trans_yz_force[1]<<std::endl;


                                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<'\t'
                                     <<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<'\t'
                                    << "Trans_force:"<<'\t'<<trans_yz_force[0]<<'\t'<<trans_yz_force[1] << '\t'
                                    << "Counter:"<<'\t'<<imp_->search_counter<<'\t'
                                    << "Deg:"<<'\t'<< 57.296 * imp_->force_direction[imp_->search_counter] <<std::endl;
                            }


                            if(count() % 10 == 0)
                            {
                                lout()<<"Current pos:"<<'\t'<< arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                                     << "Current force:" <<'\t'<< arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                                     << "Trans_force:"<<'\t'<<trans_yz_force[0]<<'\t'<<trans_yz_force[1] << '\t'
                                     << "Counter:"<<'\t'<<imp_->search_counter<< std::endl;
                            }

                        }
                        else
                        {

                            for(int i = 0; i < 2; i++)
                            {
                                imp_->v_c[i+1] = 0;
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
            //Phase 10 Maintain 2 Points Contact
            else if (imp_->phase9 && !imp_->phase10)
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

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase10 = true;
                    mout() << "Pos 10 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();
                    eeA2.getP(imp_->tpcontact_pos);

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    //return 0;

                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p10_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase10_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase10_fd[0] + 5.0))
                    {
                        arm2_final_force[0] = imp_->phase10_fd[0];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase10_fd[i] + arm2_final_force[i] - imp_->phase10_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase10_M[i];
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
            //Phase 11 allign
            else if (imp_->phase10 && !imp_->phase11)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double depth = 0;

                double desired_force[6]{ 1.0,1.0,1.0,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 20.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase 11: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                        return 0;
                    }
                }
//                if (count() % 100 == 0)
//                {
//                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << '\t' << '\t' <<"  pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
//                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
//                }

//                if (count() % 100 == 0)
//                {
//                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5]  << std::endl;
//                }

                depth = abs(arm2_current_pos[0] - imp_->tpcontact_pos[0]);

                if (depth >= 0.005 && forceCheck(arm2_final_force, desired_force, 3, 3))
                {
                    imp_->phase11 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Allign force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    mout() << "Insert Depth: " << depth << std::endl;


                    //return 0;

                }
                else
                {


                    //rz Compensation


                    mz_comp = imp_->a_y*arm2_final_force[1]-imp_->b_y*0.0825*arm2_final_force[0]+imp_->c_y;
                    arm2_final_force[5] -= mz_comp;

                    if (count() % 100 == 0)
                    {
                        mout() << "compf: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                    }

                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p11_deadzone);

//                    if (count() % 100 == 0)
//                    {
//                        mout() << "compf: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
//                    }


                    if(arm2_final_force[0] >= (imp_->phase11_fd[0] - 2) && arm2_final_force[0] <= (imp_->phase11_fd[0] + 5.5))
                    {
                        arm2_final_force[0] = imp_->phase11_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase11_fd[i] + arm2_final_force[i] - imp_->phase11_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase11_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase11_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase11_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase11_M[i + 3];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(arm2_current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, arm2_current_pos + 3, "321");


//                    mout()<<"desired pos " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
//                         << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;

                    saMove(arm2_current_pos, model_a2, 1);

                }
            }
            //Phase 12 Contact Check
            else if (imp_->phase11 && !imp_->phase12)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 20.0)
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

                if (abs(arm2_final_force[0])>=1.0)
                {
                    imp_->phase12 = true;
                    mout() << "Pos 12 Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();
                    eeA2.getP(imp_->tpcontact_pos);

                    for(int i = 0; i<6; i++)
                    {
                        imp_->v_c[i] = 0;
                    }

                    //return 0;

                }
                else
                {


                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p12_deadzone);

//                    if(arm2_final_force[0] >= (imp_->phase12_fd[0] - 2.0) && arm2_final_force[0] <= (imp_->phase12_fd[0] + 10.0))
//                    {
//                        arm2_final_force[0] = imp_->phase12_fd[0];
//                    }




                    //Impedence Controller
                    for (int i = 1; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase12_fd[i] + arm2_final_force[i] - imp_->phase12_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase12_M[i];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    arm2_current_pos[0]-=0.000008;


                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 13 Allign
            else if (imp_->phase12 && !imp_->phase13)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_final_force[i]) > 25.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                        return 0;
                    }
                }

                if (abs(arm2_final_force[4])<0.3 && abs(arm2_final_force[5])<0.3 && posCheck(arm2_current_pos, 10))
                {
                    imp_->phase13 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Allign force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                        << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    return 0;

                }
                else
                {

                    if (count() % 50 == 0)
                    {
                        mout() << "force: "<< '\t' << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << '\t'
                            << "Pos: "<<'\t'<< arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] <<std::endl;
                    }

                    forceUpperLimit(arm2_final_force, limit_area);
                    forceDeadZone(arm2_final_force, imp_->p13_deadzone);


                    if(arm2_final_force[0] >= (imp_->phase13_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase13_fd[0] + 7.5))
                    {
                        arm2_final_force[0] = imp_->phase13_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase13_fd[i] + arm2_final_force[i] - imp_->phase13_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase13_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase13_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase13_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase13_M[i + 3];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(arm2_current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, arm2_current_pos + 3, "321");

                    saMove(arm2_current_pos, model_a2, 1);

                }

            }

        }


        return 250000 - count();
    }
    HoleInPeg::HoleInPeg(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_hp\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	<Param name=\"point\" default=\"0\" abbreviation=\"p\"/>"
         "	</GroupParam>"
         "</Command>");
}
    HoleInPeg::~HoleInPeg() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(HoleInPeg)


    struct PegInHole::Imp {

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
        bool phase7 = false;
        bool phase8 = false;

        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };

        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase2_fd[6]{ 0 };
        double phase3_fd[6]{ -3.5,0,0,0,0,0 };
        double phase4_fd[6]{ -3.5,0,0,0,0,0 };
        double phase5_fd[6]{ -5.0,0,0,0,0,0 };
        double phase6_fd[6]{ 0,0,0,0,0,0 };
        double phase7_fd[6]{-3.5,0,0,0,0,0};
        double phase8_fd[6]{-3.5,0,0,0,0,0};

        //Desired Pos of Each Phase
        double phase2_xd[6]{ 0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 3500,3500,3500,0,0,0 };   //2-points
        double phase3_M[6]{ 200,100,100,0,0,0 };

        double phase4_B[6]{ 3500,3500,3500,4.5,4.5,3.5 };
        double phase4_M[6]{ 150,100,100,2.0,2.0,1.0 };

        //450 200
        double phase5_B[6]{ 3000,3500,3500,15,6.5,6.5 };
        double phase5_M[6]{ 150,100,100,3, 2.0, 2.0 };

        double phase6_B[6]{ 0,8000,8000,0,0,0 };
        double phase6_M[6]{ 0,100,100,0, 0, 0 };

        double phase7_B[6]{ 4500,4500,4500,4.5,4.5,4.5 };
        double phase7_M[6]{ 100,100,100,2.5,2.5,2.5 };

        double phase8_B[6]{ 4500,4000,4000,10,10,10 };
        double phase8_M[6]{ 100,100,100,2, 2, 2 };

        //Counter
        int contact_count = 0;
        int current_count = 0;

        //Allign Counter
        int allign_count = 0;
        int allign_success_count = 0;


        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        double current_pos_checkek[6] = {0};

        //Test
        //double actual_force[6]{ 0 };

        //Switch Angle
        double z_;
        double y_;

        //Force Buffer
        std::array<double, 20> force_buffer[6] = {};
        int buffer_index[6]{ 0 };
    };
    auto PegInHole::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto PegInHole::executeRT() -> int
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
        static double init_angle[12] =
        { 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
        0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };

        static double trigger_force[6]{ 0.5,0.5,0.5,0.0015,0.0015,0.0015 };

        static double trigger_vel[6]{ 0.000001,0.000001,0.000001,0.0001,0.0001,0.0001 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0015,0.011,0.0015 };
        static double limit_area[6]{17,8,8,0.6,0.6,0.6};

        double p3_deadzone[6]{0.1,3,3,0,0,0};
        double p4_deadzone[6]{0,0,0,0.1,0.2,0.05};
        double p5_deadzone[6]{0,0,0,0.3,0.3,0.3};

        double p7_deadzone[6]{0.5,1.0,1.0,0,0,0};
        double p8_deadzone[6]{0,1.0,1.0,0.020,0.020,0.020};

        imp_->z_ = doubleParam("z_degree");

        imp_->y_ = doubleParam("y_degree");


        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double current_force[6]{ 0 };
        double actual_force[6]{ 0 };
        double filtered_force[6]{ 0 };
        double transform_force[6]{ 0 };
        double limited_force[6]{ 0 };

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

        auto forceFilter = [&](double* actual_force_, double* filtered_force_)
        {
            for (int i = 0; i < 6; i++)
            {
                imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 20;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 20;
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
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_)
        {

            bool isValid = true;
            for (int i = 0; i < 3; i++)
            {
                if (abs(current_force_[i]) > force_check_[i] + 0.05)
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

                if ((count() - imp_->allign_count) % 30 == 0) {
                    imp_->allign_success_count++;
                    mout() << "Check " << imp_->allign_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->allign_success_count >= count_)
                    {
                        //Restore Counter
                        imp_->allign_count = 0;
                        imp_->allign_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->allign_success_count = 0;
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
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.00001)
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




        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle, current_angle + 6, current_sa_angle);



        if(!imp_->init)
        {
            double assem_pos[6]{ 0.775, 0.041748, 0.29238, PI / 2, -PI / 2, PI / 2 };
            double init_angle[6]{0};

            model_a1.setOutputPos(assem_pos);
            if(model_a1.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a1.getInputPos(init_angle);

            saJointMove(init_angle, 0);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 0, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
                master()->logFileRawName("PegInHole");

                //return 0;


            }
        }
        else
        {

            eeA1.getP(current_pos);
            eeA1.getMpm(current_pm);



            //Force Comp, Filtered, Transform
            getForceData(actual_force, 0, imp_->init);
            gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
            for (size_t i = 0; i < 6; i++)
            {
                comp_force[i] = actual_force[i] + comp_force[i];
            }

            forceFilter(comp_force, filtered_force);
            forceDeadZone(filtered_force, dead_zone);
            forceTransform(filtered_force, transform_force, 0);

            lout()<<"force"<<'\t'<<transform_force[0]<<'\t'<<transform_force[1]<<'\t'<<transform_force[2]<<'\t'
                    <<transform_force[3]<<'\t'<<transform_force[4]<<'\t'<<transform_force[5]<<'\t'
                   <<"pos"<<'\t'<<current_pos[0]<<'\t'<<current_pos[1]<<'\t'<<current_pos[2]<<'\t'
                     <<current_pos[3]<<'\t'<<current_pos[4]<<'\t'<<current_pos[5]<<std::endl;

            std::copy(transform_force, transform_force+6, limited_force);

            forceUpperLimit(limited_force, limit_area);

            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ 0.775, 0.041748, 0.29238, PI / 2, -PI / 2, PI / 2 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate Error
                double rotate_angle[3]{ 0,imp_->y_ * 2 * PI / 360, imp_->z_ * 2 * PI / 360 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");

                eeA1.setP(assem_pos);


                if (model_a1.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a1.getInputPos(assem_angle);

                saJointMove(assem_angle, 0);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {
                    imp_->phase1 = true;
                    mout() << "Assembly Start !" << std::endl;

                }

            }
            //Phase 2 Contact
            else if (imp_->phase1 && !imp_->phase2)
            {
                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a1_pm[16]{ 0 };
                eeA1.getMpm(a1_pm);
                eeA1.getP(current_pos);

                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
                }


                //Arm1
                getForceData(raw_force_checker, 0, imp_->init);
                gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) > 1.0)

                    {
                        imp_->phase2 = true;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                            << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                        break;

                    }

                }
                if (!imp_->phase2)
                {
                    current_pos[0] += 0.00001;
                    saMove(current_pos, model_a1, 0);
                }

            }
            //Phase 3 Position Move Only, Maintain Two-Point Contact
            else if (imp_->phase2 && !imp_->phase3)
            {

                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0,0,0,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 30.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (posCheck(current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos Move Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Complete force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    //return 0;
                }
                else
                {

                    if(limited_force[0] >= (imp_->phase3_fd[0] - 5.0) && limited_force[0] <= (imp_->phase3_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase3_fd[0];
                    }

                    forceDeadZone(limited_force, p3_deadzone);


                    if (count() % 100 == 0)
                    {

                        mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' <<"limited : "<<limited_force[0]<< '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << std::endl;
                    }



                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + limited_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 4 Allign
            else if (imp_->phase3 && !imp_->phase4)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0.2,0.2,0.2,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 30.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' << '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }


                if (forceCheck(transform_force, desired_force, 5))
                {
                    imp_->phase4 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Allign force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                }
                else
                {

                    if(limited_force[0] >= (imp_->phase4_fd[0] - 4.0) && limited_force[0] <= (imp_->phase4_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase4_fd[0];
                    }

                    forceDeadZone(limited_force, p4_deadzone);


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + limited_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase4_fd[i + 3] + limited_force[i + 3] - imp_->phase4_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase4_M[i + 3];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 5 Insert
            else if (imp_->phase4 && !imp_->phase5)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;
                double desired_force[6]{ 0,0,0,0,0,0 };


                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 15)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase4: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
                    mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }

                //Complete Check
                if (current_pos[0] >= 0.860)
                {
                    imp_->phase5 = true;
                    mout() << "Insert Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

                }
                else
                {

                    forceDeadZone(transform_force, p5_deadzone);

                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase5_fd[i] + transform_force[i] - imp_->phase5_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase5_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }


                    //pose
                    for (int i = 0; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase5_fd[i + 3] + transform_force[i + 3] - imp_->phase5_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase5_M[i + 3];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 6 Force to Zero
            else if(imp_->phase5 && !imp_->phase6)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0.7,0.5,0.5,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 5.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (forceCheck(transform_force, desired_force, 3))
                {
                    imp_->phase6 = true;
                    mout() << "Move Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Complete force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    //return 0;
                }
                else
                {


                    if (count() % 100 == 0)
                    {

                        mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' <<"limited : "<<limited_force[0]<< '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << std::endl;
                    }



                    //Impedence Controller
                    for (int i = 1; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase6_fd[i] + limited_force[i] - imp_->phase6_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase6_M[i];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 7 Hold Pos
            else if(imp_->phase6 && !imp_->phase7)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 10.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (posCheck(current_pos, 10))
                {
                    imp_->phase7 = true;
                    mout() << "Pos Move Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Complete force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    //return 0;
                }
                else
                {

                    if(limited_force[0] >= (imp_->phase7_fd[0] - 4.0) && limited_force[0] <= (imp_->phase7_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase7_fd[0];
                    }

                    forceDeadZone(limited_force, p7_deadzone);


                    if (count() % 100 == 0)
                    {

                        mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << std::endl;
                    }



                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase7_fd[i] + limited_force[i] - imp_->phase7_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase7_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 8 Peg Fine Allign
            else if(imp_->phase7 && !imp_->phase8)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 10.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' << '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }


                if (abs(limited_force[4])<=0.025 && abs(limited_force[5])<=0.025 && posCheck(current_pos, 10))
                {
                    imp_->phase8 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Allign force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    return 0;

                }
                else
                {

                    if(limited_force[0] >= (imp_->phase8_fd[0] - 4.0) && limited_force[0] <= (imp_->phase8_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase8_fd[0];
                    }

                    forceDeadZone(limited_force, p8_deadzone);


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase8_fd[i] + limited_force[i] - imp_->phase8_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase8_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase8_fd[i + 3] + limited_force[i + 3] - imp_->phase8_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase8_M[i + 3];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                    saMove(current_pos, model_a1, 0);
                }
            }

        }


        return 150000 - count();
    }
    PegInHole::PegInHole(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_ph\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    PegInHole::~PegInHole() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PegInHole)

    struct Insert::Imp {

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
        bool phase7 = false;

        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };

        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase2_fd[6]{ 0 };
        double phase3_fd[6]{ -4.5,0,0,0,0,0 };
        double phase4_fd[6]{ -3.5,0,0,0,0,0 };

        double phase5_fd[6]{ -3.5,0,0,0,0,0 };
        double phase6_fd[6]{ 0,0,0,0,0,0 };

        double phase8_fd[6]{-3.5,0,0,0,0,0};

        //Desired Pos of Each Phase
        double phase2_xd[6]{ 0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 1500,1500,1500,0,0,0 };
        double phase3_M[6]{ 100,100,100,0, 0, 0 };

        double phase4_B[6]{ 4500,4500,4500,4.5,4.5,4.5 };
        double phase4_M[6]{ 100,100,100,2.5,2.5,2.5 };

        //450 200
        double phase5_B[6]{ 2500,3300,3300,45,45,45 };
        double phase5_M[6]{ 100,100,100,20, 20, 20 };

        double phase6_B[6]{ 0,5000,5000,0,0,0 };
        double phase6_M[6]{ 0,100,100,0, 0, 0 };


        double phase8_B[6]{ 4500,3500,3500,10,10,10 };
        double phase8_M[6]{ 100,100,100,2, 2, 2 };

        //Counter
        int contact_count = 0;
        int current_count = 0;

        //Allign Counter
        int allign_count = 0;
        int allign_success_count = 0;


        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        double current_pos_checkek[6] = {0};

        //Test
        //double actual_force[6]{ 0 };

        //Switch Model
        int m_;
        int y_;
        int z_;

        //Force Buffer
        std::array<double, 20> force_buffer[6] = {};
        int buffer_index[6]{ 0 };
    };
    auto Insert::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto Insert::executeRT() -> int
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
        static double init_angle[12] =
        { 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
        0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };

        static double trigger_force[6]{ 0.5,0.5,0.5,0.0015,0.0015,0.0015 };

        static double trigger_vel[6]{ 0.000001,0.000001,0.000001,0.0001,0.0001,0.0001 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0015,0.011,0.0015 };
        static double limit_area[6]{17,8,8,0.6,0.6,0.6};

        double p3_deadzone[6]{0,10,10,0,0,0};


        double p4_deadzone[6]{0.5,1.0,1.0,0,0,0};

        double p5_deadzone[6]{0,0,0,0.1,0.1,0.1};

        double p8_deadzone[6]{0,1.0,1.0,0.020,0.020,0.020};


        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double current_force[6]{ 0 };
        double actual_force[6]{ 0 };
        double filtered_force[6]{ 0 };
        double transform_force[6]{ 0 };
        double limited_force[6]{ 0 };

        imp_->z_ = doubleParam("z_degree");

        imp_->y_ = doubleParam("y_degree");



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


        auto forceFilter = [&](double* actual_force_, double* filtered_force_)
        {
            for (int i = 0; i < 6; i++)
            {
                imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 20;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 20;
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
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_)
        {

            bool isValid = true;
            for (int i = 0; i < 3; i++)
            {
                if (abs(current_force_[i]) > force_check_[i] + 0.05)
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

                if ((count() - imp_->allign_count) % 30 == 0) {
                    imp_->allign_success_count++;
                    mout() << "Check " << imp_->allign_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->allign_success_count >= count_)
                    {
                        //Restore Counter
                        imp_->allign_count = 0;
                        imp_->allign_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->allign_success_count = 0;
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
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.00001)
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




        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle, current_angle + 6, current_sa_angle);



        if(!imp_->init)
        {
            double assem_pos[6]{ 0.800, 0.041748, 0.29238, PI / 2, -PI / 2, PI / 2 };
            double init_angle[6]{0};

            model_a1.setOutputPos(assem_pos);
            if(model_a1.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a1.getInputPos(init_angle);

            saJointMove(init_angle, 0);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 0, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
//                imp_->phase1 = true;
//                imp_->phase2 = true;
//                imp_->phase3 = true;
//                imp_->phase4 = true;

                //return 0;


            }
        }
        else
        {

            eeA1.getP(current_pos);
            eeA1.getMpm(current_pm);



            //Force Comp, Filtered, Transform
            getForceData(actual_force, 0, imp_->init);
            gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
            for (size_t i = 0; i < 6; i++)
            {
                comp_force[i] = actual_force[i] + comp_force[i];
            }

            forceFilter(comp_force, filtered_force);
            forceDeadZone(filtered_force, dead_zone);
            forceTransform(filtered_force, transform_force, 0);

            std::copy(transform_force, transform_force+6, limited_force);

            forceUpperLimit(limited_force, limit_area);

            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ 0.800, 0.041748, 0.29238, PI / 2, -PI / 2, PI / 2 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate Error
                double rotate_angle[3]{ 0,imp_->y_ * 2 * PI / 360, imp_->z_ * 2 * PI / 360 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");

                eeA1.setP(assem_pos);


                if (model_a1.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a1.getInputPos(assem_angle);

                saJointMove(assem_angle, 0);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {
                    imp_->phase1 = true;
                    mout() << "Assembly Start !" << std::endl;
                    //return 0;

                }
            }
            //Phase 5 Insert
            else if(imp_->phase1 && !imp_->phase2)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;
                double desired_force[6]{ 0,0,0,0,0,0 };


                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 5.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase5: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
                    mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }

                //Complete Check
                if (current_pos[0] >= 0.860)
                {
                    imp_->phase2 = true;
                    mout() << "Insert Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

                }
                else
                {

                    forceDeadZone(transform_force, p5_deadzone);

                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase5_fd[i] + transform_force[i] - imp_->phase5_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase5_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }


                    //pose
                    for (int i = 0; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase5_fd[i + 3] + transform_force[i + 3] - imp_->phase5_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase5_M[i + 3];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    //aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 6 Force to Zero
            else if(imp_->phase2 && !imp_->phase3)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0.7,0.5,0.5,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 5.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (forceCheck(transform_force, desired_force, 3))
                {
                    imp_->phase3 = true;
                    mout() << "Move Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Complete force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    return 0;
                }
                else
                {


                    if (count() % 100 == 0)
                    {

                        mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' <<"limited : "<<limited_force[0]<< '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << std::endl;
                    }



                    //Impedence Controller
                    for (int i = 1; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase6_fd[i] + limited_force[i] - imp_->phase6_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase6_M[i];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 7 Hold Pos
            else if(imp_->phase3 && !imp_->phase4)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 15.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (posCheck(current_pos, 10))
                {
                    imp_->phase4 = true;
                    mout() << "Pos Move Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Complete force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    //return 0;
                }
                else
                {

                    if(limited_force[0] >= (imp_->phase4_fd[0] - 4.0) && limited_force[0] <= (imp_->phase4_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase4_fd[0];
                    }

                    forceDeadZone(limited_force, p4_deadzone);


                    if (count() % 100 == 0)
                    {

                        mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << std::endl;
                    }



                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + limited_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 8 Peg Fine Allign
            else if(imp_->phase4 && !imp_->phase5)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 10.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' << '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }


                if (abs(limited_force[4])<=0.025 && abs(limited_force[5])<=0.025 && posCheck(current_pos, 10))
                {
                    imp_->phase5 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Allign force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    return 0;

                }
                else
                {

                    if(limited_force[0] >= (imp_->phase8_fd[0] - 4.0) && limited_force[0] <= (imp_->phase8_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase8_fd[0];
                    }

                    forceDeadZone(limited_force, p8_deadzone);


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase8_fd[i] + limited_force[i] - imp_->phase8_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase8_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase8_fd[i + 3] + limited_force[i + 3] - imp_->phase8_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase8_M[i + 3];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                    saMove(current_pos, model_a1, 0);
                }
            }

        }


        return 80000 - count();
    }
    Insert::Insert(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_insert\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    Insert::~Insert() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Insert)



    ARIS_REGISTRATION {
        aris::core::class_<HoleInPeg>("HoleInPeg")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PegInHole>("PegInHole")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Insert>("Insert")
            .inherit<aris::plan::Plan>();
    }
}
