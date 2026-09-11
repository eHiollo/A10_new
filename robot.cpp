#include "robot.hpp"
#include "gravcomp.hpp"
#include <array>
#include <algorithm>
#include <ostream>
#include <unordered_map>
#include <vector>
#include "a10_tcp_server.hpp"
#include "gripper.hpp"

extern A10TcpServer *g_tcp_server;
extern BusServo *g_gripper;


using namespace std;

namespace robot
{

    //example
    struct ModelSetPos::Imp {
		//Switch Model
		int m_;

		double arm1_p_vector[9]{ 0 };
		double arm1_l_vector[9]{ 0 };
		double arm2_p_vector[9]{ 0 };
		double arm2_l_vector[9]{ 0 };

		double arm1_init_force[9]{ 0 };
		double arm2_init_force[9]{ 0 };

		bool init = false;
		bool contact_check = false;

		//Force Buffer
		std::array<double, 10> force_buffer[6] = {};
		int buffer_index[6]{ 0 };

	};
    auto ModelSetPos::prepareNrt()->void
        {
            for (auto& m : motorOptions()) m = aris::plan::Plan::CHECK_NONE |
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }
    auto ModelSetPos::executeRT()->int
        {
		if (!require_dual_arm(*this, "m_set"))
		{
			return 0;
		}
		//test for get force data
		imp_->m_ = int32Param("model");
		GravComp gc;

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

 
		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
				{
					mout() << "Force Sensor Error" << std::endl;
				}


				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				if (m_ == 0)
				{
					mout() << "Compensate Init Force A1" << std::endl;
					mout() << "Init 1 : " << imp_->arm1_init_force[0] << '\t' << imp_->arm1_init_force[1] << '\t' << imp_->arm1_init_force[2] << '\t'
						<< imp_->arm1_init_force[3] << '\t' << imp_->arm1_init_force[4] << '\t' << imp_->arm1_init_force[5] << std::endl;
				}
				else if (m_ == 1)
				{
					mout() << "Compensate Init Force A2" << std::endl;
					mout() << "Init 2 : " << imp_->arm2_init_force[0] << '\t' << imp_->arm2_init_force[1] << '\t' << imp_->arm2_init_force[2] << '\t'
						<< imp_->arm2_init_force[3] << '\t' << imp_->arm2_init_force[4] << '\t' << imp_->arm2_init_force[5] << std::endl;
				}




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

			}
		};

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};

		if (!imp_->init && !imp_->contact_check)
		{
			getForceData(imp_->arm1_init_force, 0, imp_->init);
			getForceData(imp_->arm2_init_force, 1, imp_->init);

			double current_angle[12] = { 0 };

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].actualPos();
			}

			dualArm.setInputPos(current_angle);
			if (dualArm.forwardKinematics())
			{
				mout() << "Forward failed" << std::endl;
				return 0;
			}

			imp_->init = true;
			mout() << "Init" << std::endl;

		}
		else if (imp_->init && !imp_->contact_check)
		{
			double raw_force_checker[12]{ 0 };
			double comp_force_checker[12]{ 0 };
			double force_checker[12]{ 0 };

			double a1_pm[16]{ 0 };
			double a2_pm[16]{ 0 };

			eeA1.getMpm(a1_pm);
			eeA2.getMpm(a2_pm);


			//Arm1
			getForceData(raw_force_checker, 0, imp_->init);
			gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
			//Arm2
			getForceData(raw_force_checker + 6, 1, imp_->init);
			gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);


			for (int i = 0; i < 12; i++)
			{
				force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

				if (abs(force_checker[i]) > 3)
				{
					imp_->contact_check = true;
					mout() << "Contact Check" << std::endl;
					break;
				}
			}
		}
		else
		{
			double current_pm[16]{ 0 };


			if (imp_->m_ == 0)
			{

				eeA1.getMpm(current_pm);

				double raw_force[6]{ 0 };
				double arm1_compf[6]{ 0 };
				double arm1_actual_force[6]{ 0 };
				double arm1_filtered_force[6]{ 0 };


				getForceData(raw_force, 0, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_compf);

				for (int i = 0; i < 6; i++)
				{
					arm1_actual_force[i] = arm1_compf[i] + raw_force[i];
				}
				
				forceFilter(arm1_actual_force, arm1_filtered_force);

				if (count() % 100 == 0)
				{
					mout() << "A1_Force: " << arm1_filtered_force[0] << '\t' << arm1_filtered_force[1] << '\t' << arm1_filtered_force[2]
						<< '\t' << arm1_filtered_force[3] << '\t' << arm1_filtered_force[4] << '\t' << arm1_filtered_force[5] << std::endl;
				}


			}
			else if (imp_->m_ == 1)
			{

				eeA2.getMpm(current_pm);

				double raw_force[6]{ 0 };
				double arm2_compf[6]{ 0 };
				double arm2_actual_force[6]{ 0 };
				double arm2_filtered_force[6]{ 0 };


				getForceData(raw_force, 1, imp_->init);
				gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_compf);

				for (int i = 0; i < 6; i++)
				{
					arm2_actual_force[i] = arm2_compf[i] + raw_force[i];
				}

				forceFilter(arm2_actual_force, arm2_filtered_force);

				if (count() % 100 == 0)
				{
					mout() << "A2_Force: " << arm2_filtered_force[0] << '\t' << arm2_filtered_force[1] << '\t' << arm2_filtered_force[2]
						<< '\t' << arm2_filtered_force[3] << '\t' << arm2_filtered_force[4] << '\t' << arm2_filtered_force[5] << std::endl;
				}
			}
			else
			{
				mout() << "Model Error" << std::endl;
				return 0;
			}
		}


		return 30000 - count();
       

        return 0;

        }
    ModelSetPos::ModelSetPos(const std::string& name)
        {
            aris::core::fromXmlString(command(),
            "<Command name=\"m_set\">"
            "	<GroupParam>"
            "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
            "	</GroupParam>"
            "</Command>");
        }
    ModelSetPos::~ModelSetPos() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(ModelSetPos)


	struct ModelInit::Imp{

        // m_init：preset 0..5 对应六组初始关节目标
        int preset_index = 0;

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

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

    };
    auto ModelInit::prepareNrt()->void {

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        imp_->init = false;

        GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;



    }
    auto ModelInit::executeRT()->int {

        
        //dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		auto& arm1 = dualArm.subModels().at(0);
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		const bool dual = submodel_count(*this) >= 2;
		aris::dynamic::Model* model_a2 = nullptr;
		aris::dynamic::GeneralMotion* eeA2 = nullptr;
		if (dual)
		{
			model_a2 = &dynamic_cast<aris::dynamic::Model&>(dualArm.subModels().at(1));
			eeA2 = &dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2->generalMotionPool().at(0));
		}

        double eePos[12] = { 0 };

        static double move = 0.0001;
        static double tolerance = 0.00009;
		(void)tolerance;

        GravComp gc;

        // static double init_pos[12] =
        // { 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
        // 0, 0, -2 * PI / 3, PI / 6, PI / 2, 0 };

        //让主臂和从一样
        const double kDeg = PI / 180.0;
        static const double k_init_preset_0[12] = {
            -117.127 * kDeg, -46.664 * kDeg, 136.691 * kDeg, -37.318 * kDeg, -70.145 * kDeg, -13.986 * kDeg,
            0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, 0
        };
        static const double k_init_preset_1[12] = {
            -19.5 * kDeg, -19.3 * kDeg, 126.5 * kDeg, -61 * kDeg, -70 * kDeg, -9 * kDeg,
            0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, 0
        };
        static const double k_init_preset_2[12] = {
            -31 * kDeg, -23.5 * kDeg, 123.5 * kDeg, -51.6 * kDeg, -62.4 * kDeg, -22 * kDeg,
            0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, 0
        };
        static const double k_init_preset_3[12] = {
            -40 * kDeg, -8.8 * kDeg, 106.5 * kDeg, -32.8 * kDeg, -54.3 * kDeg, -33 * kDeg,
            0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, 0
        };
        static const double k_init_preset_4[12] = {
            -11.7 * kDeg, -7.7 * kDeg, 95.6 * kDeg, -15.8 * kDeg, -86.3 * kDeg, -2.5 * kDeg,
            0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, 0
        };
        static const double k_init_preset_5[12] = {
            -7.8 * kDeg, 1.6 * kDeg, 89.7 * kDeg, -18.6 * kDeg, -88.7 * kDeg, 0.9 * kDeg,
            0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, 0
        };

        auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };
			if (!force_slave_ok(*this, m_))
			{
				return;
			}

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

        static const double *const k_init_preset_table[6] = {
            k_init_preset_0, k_init_preset_1, k_init_preset_2, k_init_preset_3,
            k_init_preset_4, k_init_preset_5};

        if (count() == 1) {
            int p = int32Param("preset");
            if (p < 0 || p > 5) {
                mout() << "m_init: preset must be 0..5, got " << p << ", using 0" << std::endl;
                p = 0;
            }
            imp_->preset_index = p;
            mout() << "m_init: preset=" << imp_->preset_index << std::endl;

            getForceData(imp_->arm1_init_force, 0, imp_->init);
            if (dual)
            {
                getForceData(imp_->arm2_init_force, 1, imp_->init);
            }
            if (force_slave_ok(*this, 0))
            {
                master()->logFileRawName("log/forceComp_");
            }
            imp_->init = true;
        }

        const double *const init_pos = k_init_preset_table[imp_->preset_index];

        //6-12维反向
        // static double init_pos[12] =
        // { 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
        // 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

        // //1-6维反向
        // static double init_pos[12] =
        // { 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0,
        // 0, 0, -2 * PI / 3, PI / 6, PI / 2, 0 };

        double current_arm1_pm[16]{0};
        double current_arm1_force[6]{0};
        
        double current_arm2_pm[16]{0};
        double current_arm2_force[6]{0};

        double arm1_comp_force[6]{0};
        double arm2_comp_force[6]{0};

        double arm1_actual_force[6]{0};
        double arm2_actual_force[6]{0};

        double arm1_raw_force[6]{0};
        double arm2_raw_force[6]{0};

        eeA1.getMpm(current_arm1_pm);
		if (eeA2 != nullptr)
		{
			eeA2->getMpm(current_arm2_pm);
		}

		if (force_slave_ok(*this, 0))
		{
			getForceData(current_arm1_force, 0, imp_->init);
		}
        if (dual && force_slave_ok(*this, 1))
        {
            getForceData(current_arm2_force, 1, imp_->init);
        }
		
        gc.getCompFT(current_arm1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);
        gc.getCompFT(current_arm2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

		for (int i = 0; i < 6; i++)
		{
            arm1_actual_force[i] = arm1_comp_force[i] + current_arm1_force[i];
            arm2_actual_force[i] = arm2_comp_force[i] + current_arm2_force[i];
		}

        for (int i = 0; i < 6; i++)
        {
            arm1_raw_force[i] = current_arm1_force[i];
            arm2_raw_force[i] = current_arm2_force[i];
        }

        // if(count()%50==0)
        // {
        //     lout()<<arm1_raw_force[0]<<'\t'<<arm1_raw_force[1]<<'\t'<<arm1_raw_force[2]<<'\t'
        //     <<arm1_raw_force[3]<<'\t'<<arm1_raw_force[4]<<'\t'<<arm1_raw_force[5]<<'\t'
        //     <<arm1_actual_force[0]<<'\t'<<arm1_actual_force[1]<<'\t'<<arm1_actual_force[2]<<'\t'
        //     <<arm1_actual_force[3]<<'\t'<<arm1_actual_force[4]<<'\t'<<arm1_actual_force[5]<<'\t'
        //     <<arm2_raw_force[0]<<'\t'<<arm2_raw_force[1]<<'\t'<<arm2_raw_force[2]<<'\t'
        //     <<arm2_raw_force[3]<<'\t'<<arm2_raw_force[4]<<'\t'<<arm2_raw_force[5]<<'\t'
        //     <<arm2_actual_force[0]<<'\t'<<arm2_actual_force[1]<<'\t'<<arm2_actual_force[2]<<'\t'
        //     <<arm2_actual_force[3]<<'\t'<<arm2_actual_force[4]<<'\t'<<arm2_actual_force[5]<<std::endl;
        // }
        
        modelBase()->setInputPos(init_pos);

        if (modelBase()->forwardDynamics())
        {
            throw std::runtime_error("Forward Kinematics Position Failed!");
        }


        const int n_axis = axis_count(*this);
        double current_angle[12] = { 0 };

        for (int i = 0; i < n_axis; i++)
        {
            current_angle[i] = controller()->motorPool()[i].targetPos();
        }
        if (dual)
        {
            for (int i = 6; i < 12 && static_cast<aris::Size>(i) < motor_count(*this); i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }
        }


        auto motorsPositionCheck = [=]()
        {
            const int n_check = dual ? 12 : n_axis;
            for(int i = 0; i < n_check; i++)
            {
                if(std::fabs(current_angle[i]-init_pos[i])>=move)
                {
                    return false;
                }
            }

            return true;
        };



        const int n_move = dual ? 12 : n_axis;
        for (int i = 0; i < n_move; i++)
        {
            if (static_cast<aris::Size>(i) >= motor_count(*this))
            {
                break;
            }
            if (current_angle[i] <= init_pos[i] - move)
            {
                controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
            }
            else if (current_angle[i] >= init_pos[i] + move)
            {
                controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
            }
        }





        if (count() % 1000 == 0)
        {
            mout()<<current_angle[0]<< "\t"<<current_angle[1]<< "\t"<<current_angle[2]<< "\t"<<current_angle[3]<< "\t"<<current_angle[4]<< "\t"
            <<current_angle[5]<< "\t"<<current_angle[6]<< "\t"<<current_angle[7]<< "\t"<<current_angle[8]<< "\t"<<current_angle[9]<< "\t"
            <<current_angle[10]<< "\t"<<current_angle[11]<<std::endl;
        }


        if (motorsPositionCheck())
        {

            mout()<<"Back to Init Position"<<std::endl;
            modelBase()->setInputPos(current_angle);
            if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;

            mout()<<"current angle: \n"<<current_angle[0]<< "\t"<<current_angle[1]<< "\t"<<current_angle[2]<< "\t"<<current_angle[3]<< "\t"<<current_angle[4]<< "\t"
            <<current_angle[5]<< "\t"<<current_angle[6]<< "\t"<<current_angle[7]<< "\t"<<current_angle[8]<< "\t"<<current_angle[9]<< "\t"
            <<current_angle[10]<< "\t"<<current_angle[11]<<std::endl;

            modelBase()->getOutputPos(eePos);
            mout() << "current end position: \n" <<eePos[0]<< "\t"<<eePos[1]<< "\t"<<eePos[2]<< "\t"<<eePos[3]<< "\t"<<eePos[4]<< "\t"<<eePos[5]<< "\t"
            <<eePos[6]<< "\t"<<eePos[7]<< "\t"<<eePos[8]<< "\t"<<eePos[9]<< "\t"<<eePos[10]<< "\t"<<eePos[11]<<std::endl;


            return 0;
        }
        else
        {
            if(count()==28000)
            {
                mout()<<"Over Time"<<std::endl;
            }

            return 28000 - count();
        }
    }
    ModelInit::ModelInit(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"m_init\">"
            "  <GroupParam>"
            "    <Param name=\"preset\" abbreviation=\"p\" default=\"0\"/>"
            "  </GroupParam>"
            "</Command>");
    }
    ModelInit::~ModelInit() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(ModelInit)


    struct ModelGet::Imp {
        //Switch Model
        int m_;

        double arm1_p_vector[6]{0};
        double arm1_l_vector[6]{0};
        double arm2_p_vector[6]{0};
        double arm2_l_vector[6]{0};

        double arm1_init_force[6]{0};
        double arm2_init_force[6]{0};

        bool init = false;

		//Force Buffer
		std::array<double, 10> force_buffer[6] = {};
		int buffer_index[6]{ 0 };

    };
	auto ModelGet::prepareNrt() -> void
	{
        for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE;
        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
        imp_->init = true;
	}
	auto ModelGet::executeRT() -> int
	{
        if (!require_dual_arm(*this, "m_get"))
        {
            return 0;
        }
		//test for get force data
        imp_->m_ = int32Param("model");
        GravComp gc;

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


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                {
                    mout() << "Force Sensor Error" << std::endl;
                }


                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            // m_get 用于验证 gravcomp：这里返回原始传感器数据（不做 init_force 减法）
            static_cast<void>(init_);
        };

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
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

      
        double current_pm[16]{0};


        if(imp_->m_ == 0)
        {

            eeA1.getMpm(current_pm);

            double raw_force[6]{0};
            double arm1_compf[6]{0};
            double arm1_actual_force[6]{0};
            double filtered_force[6]{ 0 };
            double transform_force[6]{ 0 };

            getForceData(raw_force, 0, imp_->init);
            gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_compf);

            for(int i = 0; i<6; i++)
            {
                arm1_actual_force[i] = arm1_compf[i] + raw_force[i];
            }

            forceFilter(arm1_actual_force, filtered_force);
            forceTransform(filtered_force, transform_force, imp_->m_);

            if(count()%100 == 0)
            {
                mout()<<"A1_Force: "<< transform_force[0]<<'\t'<< transform_force[1]<<'\t'<< transform_force[2]
                     <<'\t'<< transform_force[3]<<'\t'<< transform_force[4]<<'\t'<< transform_force[5]<<std::endl;
            }


        }
        else if(imp_->m_ == 1)
        {

            eeA2.getMpm(current_pm);

            double raw_force[6]{0};
            double arm2_compf[6]{0};
            double arm2_actual_force[6]{0};
            double filtered_force[6]{ 0 };
            double transform_force[6]{ 0 };

            getForceData(raw_force, 1, imp_->init);
            gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_compf);



            for(int i = 0; i<6; i++)
            {
                arm2_actual_force[i] = arm2_compf[i] + raw_force[i];
            }

            forceFilter(arm2_actual_force, filtered_force);
            forceTransform(filtered_force, transform_force, imp_->m_);
            if(count()%100 == 0)
            {
                mout()<<"A2_Force: "<< transform_force[0]<<'\t'<< transform_force[1]<<'\t'<< transform_force[2]
                     <<'\t'<< transform_force[3]<<'\t'<< transform_force[4]<<'\t'<< transform_force[5]<<std::endl;
                mout()<<"raw_force: "<< raw_force[0]<<'\t'<< raw_force[1]<<'\t'<< raw_force[2]
                     <<'\t'<< raw_force[3]<<'\t'<< raw_force[4]<<'\t'<< raw_force[5]<<std::endl;
                mout()<<"arm2_compf: "<< arm2_compf[0]<<'\t'<< arm2_compf[1]<<'\t'<< arm2_compf[2]
                     <<'\t'<< arm2_compf[3]<<'\t'<< arm2_compf[4]<<'\t'<< arm2_compf[5]<<std::endl;               
                mout()<<"arm2_actual_force: "<< arm2_actual_force[0]<<'\t'<< arm2_actual_force[1]<<'\t'<< arm2_actual_force[2]
                     <<'\t'<< arm2_actual_force[3]<<'\t'<< arm2_actual_force[4]<<'\t'<< arm2_actual_force[5]<<std::endl;
                mout()<<"filtered_force: "<< filtered_force[0]<<'\t'<< filtered_force[1]<<'\t'<< filtered_force[2]
                     <<'\t'<< filtered_force[3]<<'\t'<< filtered_force[4]<<'\t'<< filtered_force[5]<<std::endl;
            }
        }
        else if (imp_->m_ == 2)
        {
            double a1_comp_force[6]{ 0 };
            double a1_current_pm[16]{ 0 };
            double a1_current_force[6]{ 0 };
            double a1_actual_force[6]{ 0 };
            double a1_transform_force[6]{ 0 };

            double a2_comp_force[6]{ 0 };
            double a2_current_pm[16]{ 0 };
            double a2_current_force[6]{ 0 };
            double a2_actual_force[6]{ 0 };
            double a2_transform_force[6]{ 0 };


            GravComp gc;

            getForceData(a1_current_force, 0, imp_->init);
            eeA1.getMpm(a1_current_pm);
            gc.getCompFT(a1_current_pm,imp_->arm1_l_vector,imp_->arm1_p_vector, a1_comp_force);

            for(int i = 0; i<6; i++)
            {
                a1_actual_force[i] = a1_current_force[i] + a1_comp_force[i];
            }

            forceTransform(a1_actual_force, a1_transform_force, 0);

            getForceData(a2_current_force, 1, imp_->init);
            eeA2.getMpm(a2_current_pm);
            gc.getCompFT(a2_current_pm,imp_->arm2_l_vector,imp_->arm2_p_vector, a2_comp_force);

            for(int i = 0; i<6; i++)
            {
                a2_actual_force[i] = a2_current_force[i] + a2_comp_force[i];
            }

            forceTransform(a2_actual_force, a2_transform_force, 1);

            if(count() % 50 == 0)
            {
                mout()<<"A1_Force"<<"\t"<<a1_transform_force[0]<<"\t"<<a1_transform_force[1]<<"\t"<<a1_transform_force[2]<<"\t"
                        <<a1_transform_force[3]<<"\t"<<a1_transform_force[4]<<"\t"<<a1_transform_force[5]<<"\t"<<"\t"
                       <<"A2_Force"<<"\t"<<a2_transform_force[0]<<"\t"<<a2_transform_force[1]<<"\t"<<a2_transform_force[2]<<"\t"
                       <<a2_transform_force[3]<<"\t"<<a2_transform_force[4]<<"\t"<<a2_transform_force[5]<<std::endl;
            }

        }

        else
        {
            mout()<<"Model Error"<<std::endl;
            return 0;
        }


        return 1500-count();
	}
	ModelGet::ModelGet(const std::string& name)
	{
        aris::core::fromXmlString(command(),
        "<Command name=\"m_get\">"
        "	<GroupParam>"
        "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
        "	</GroupParam>"
        "</Command>");
	}
	ModelGet::~ModelGet() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ModelGet)


	auto ModelForward::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelForward::executeRT()->int
	{
		TCurve2 s1(1.0, 3.0, 20.0);
		s1.getCurveParam();
		static double init_pos[12]{};
		double input_angle[12]{};
		double ee_pos[12]{};

        if (count() == 1)
        {
            double begin_angle[12]{ 0 };
            const int n_axis = axis_count(*this);
            for (int i = 0; i < n_axis; i++)
            {
                begin_angle[i] = controller()->motorPool()[i].targetPos();
            }

            aris::dynamic::dsp(1, n_axis, begin_angle);

			//this->master()->logFileRawName("move");
			mout() << "read init angle" << std::endl;

			modelBase()->setInputPos(begin_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			mout() << "input" << std::endl;

			modelBase()->getOutputPos(init_pos);

			aris::dynamic::dsp(1, 12, init_pos);

		}



		ee_pos[0] = init_pos[0] + 0.01 * s1.getTCurve(count());
		ee_pos[1] = init_pos[1] - 0.01 * s1.getTCurve(count());

		ee_pos[2] = init_pos[2];
		ee_pos[3] = init_pos[3];
		ee_pos[4] = init_pos[4];
		ee_pos[5] = init_pos[5];

		ee_pos[6] = init_pos[6];
		ee_pos[7] = init_pos[7];
		ee_pos[8] = init_pos[8];
		ee_pos[9] = init_pos[9];
		ee_pos[10] = init_pos[10];
		ee_pos[11] = init_pos[11];


		modelBase()->setOutputPos(ee_pos);
		if (modelBase()->inverseKinematics())
		{
			throw std::runtime_error("Inverse Kinematics Position Failed!");
		}



		modelBase()->getInputPos(input_angle);

		const int n_axis = axis_count(*this);
		for (int i = 0; i < n_axis; i++)
		{
			controller()->motorPool()[i].setTargetPos(input_angle[i]);
		}

		if (count() % 100 == 0)
		{
			mout() << "Arm1:" << input_angle[0] * 180 / PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
				<< input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI << "\n"
				<< "Arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
				<< input_angle[9] * 180 / PI << "\t" << input_angle[10] * 180 / PI << "\t" << input_angle[11] * 180 / PI << "\t" << count() << std::endl;
		}



		return 10000 - count();

	}
	ModelForward::ModelForward(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_forward\">"
			"</Command>");
	}
	ModelForward::~ModelForward() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ModelForward)


    struct ModelTest::Imp {

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

        //Switch Model
        int m_;

    };
    auto ModelTest::prepareNrt()->void
	{

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


        GravComp gc;
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


        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;

        getForceData(imp_->arm1_init_force, 0, imp_->init);
        getForceData(imp_->arm2_init_force, 1, imp_->init);

        imp_->init = true;

        mout()<<"Get Init Force!"<<std::endl;


	}
    auto ModelTest::executeRT()->int
	{
        if (!require_dual_arm(*this, "m_t"))
        {
            return 0;
        }

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


        double a1_comp_force[6]{ 0 };
        double a1_current_pm[16]{ 0 };
        double a1_current_force[6]{ 0 };
        double a1_actual_force[6]{ 0 };
        double a1_transform_force[6]{ 0 };

        double a2_comp_force[6]{ 0 };
        double a2_current_pm[16]{ 0 };
        double a2_current_force[6]{ 0 };
        double a2_actual_force[6]{ 0 };
        double a2_transform_force[6]{ 0 };


        GravComp gc;

        getForceData(a1_current_force, 0, imp_->init);
        eeA1.getMpm(a1_current_pm);
        gc.getCompFT(a1_current_pm,imp_->arm1_l_vector,imp_->arm1_p_vector, a1_comp_force);

        for(int i = 0; i<6; i++)
        {
            a1_actual_force[i] = a1_current_force[i] + a1_comp_force[i];
        }

        forceTransform(a1_actual_force, a1_transform_force, 0);

        getForceData(a2_current_force, 1, imp_->init);
        eeA2.getMpm(a2_current_pm);
        gc.getCompFT(a2_current_pm,imp_->arm2_l_vector,imp_->arm2_p_vector, a2_comp_force);

        for(int i = 0; i<6; i++)
        {
            a2_actual_force[i] = a2_current_force[i] + a2_comp_force[i];
        }

        forceTransform(a2_actual_force, a2_transform_force, 1);

        if(count() % 50 == 0)
        {
            mout()<<"A1_Force"<<"\t"<<a1_transform_force[0]<<"\t"<<a1_transform_force[1]<<"\t"<<a1_transform_force[2]<<"\t"
                    <<a1_transform_force[3]<<"\t"<<a1_transform_force[4]<<"\t"<<a1_transform_force[5]<<"\t"<<"\t"
                   <<"A2_Force"<<"\t"<<a2_transform_force[0]<<"\t"<<a2_transform_force[1]<<"\t"<<a2_transform_force[2]<<"\t"
                   <<a2_transform_force[3]<<"\t"<<a2_transform_force[4]<<"\t"<<a2_transform_force[5]<<std::endl;
        }




        return 1500 - count();

	}
    ModelTest::ModelTest(const std::string& name)
	{

        aris::core::fromXmlString(command(),
            "<Command name=\"m_t\">"
            "	<GroupParam>"
            "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
            "	</GroupParam>"
            "</Command>");
	}
    ModelTest::~ModelTest() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(ModelTest)


//Force comp
	struct ModelComP::Imp {
		bool target1_reached = false;
		bool target2_reached = false;
		bool target3_reached = false;
		bool target4_reached = false;
		bool target5_reached = false;
		bool target6_reached = false;  // 回到初始位置

		bool init = false;

		bool stop_flag = false;
		int stop_count = 0;
		int stop_time = 2200;
		int current_stop_time = 0;

		int accumulation_count = 0;

		//temp data to stroage 10 times of actual force

		// 姿态数量
		static constexpr int NUM_POSES = 5;

		// For Arm 1
		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		double arm1_temp_force_1[6] = { 0 };
		double arm1_temp_force_2[6] = { 0 };
		double arm1_temp_force_3[6] = { 0 };
		double arm1_temp_force_4[6] = { 0 };
		double arm1_temp_force_5[6] = { 0 };

		double arm1_force_data_1[6] = { 0 };
		double arm1_force_data_2[6] = { 0 };
		double arm1_force_data_3[6] = { 0 };
		double arm1_force_data_4[6] = { 0 };
		double arm1_force_data_5[6] = { 0 };

		double arm1_ee_pm_1[16]{ 0 };
		double arm1_ee_pm_2[16]{ 0 };
		double arm1_ee_pm_3[16]{ 0 };
		double arm1_ee_pm_4[16]{ 0 };
		double arm1_ee_pm_5[16]{ 0 };

		double arm1_comp_f[6]{ 0 };
		double arm1_init_force[6]{ 0 };


		//For Arm 2
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		double arm2_temp_force_1[6] = { 0 };
		double arm2_temp_force_2[6] = { 0 };
		double arm2_temp_force_3[6] = { 0 };
		double arm2_temp_force_4[6] = { 0 };
		double arm2_temp_force_5[6] = { 0 };

		double arm2_force_data_1[6] = { 0 };
		double arm2_force_data_2[6] = { 0 };
		double arm2_force_data_3[6] = { 0 };
		double arm2_force_data_4[6] = { 0 };
		double arm2_force_data_5[6] = { 0 };

		double arm2_ee_pm_1[16]{ 0 };
		double arm2_ee_pm_2[16]{ 0 };
		double arm2_ee_pm_3[16]{ 0 };
		double arm2_ee_pm_4[16]{ 0 };
		double arm2_ee_pm_5[16]{ 0 };

		double arm2_comp_f[6]{ 0 };
		double arm2_init_force[6]{ 0 };

	};
	auto ModelComP::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		// 读取上一次标定结果，作为“姿态退化/标定失败”时的兜底（避免把文件写坏）
		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "ModelComP: load existing P & L Vector (fallback)" << std::endl;



	}
	auto ModelComP::executeRT()->int
	{
        if (!require_dual_arm(*this, "m_comp"))
        {
            return 0;
        }

        GravComp gc;
		static double tolerance = 0.0001;

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


		// Only One Arm Move Each Command
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


        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00008;

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

		//Ethercat Warning
        // 修复：标定时使用原始力数据，不减去init_force
        // 重力补偿算法本身会计算传感器零偏（F0），所以标定数据必须是原始值
		auto getForceData = [&](double* data_, int m_, bool init_)
		{
			int raw_force[6]{ 0 };

            for (int i = 0; i < 6; ++i)
			{
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
				{
                    mout() << "Force Sensor Error" << std::endl;
				}
				// 直接使用原始力数据（用于标定）
				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);
			}
            
			if (!init_)
			{
                if(m_ == 0)
                {
                     mout() << "Compensate Init Force A1" << std::endl;
                     mout()<<"Init 1 : "<<imp_->arm1_init_force[0]<<'\t'<<imp_->arm1_init_force[1]<<'\t'<<imp_->arm1_init_force[2]<<'\t'
                             <<imp_->arm1_init_force[3]<<'\t'<<imp_->arm1_init_force[4]<<'\t'<<imp_->arm1_init_force[5]<<std::endl;
                }
                else if (m_ == 1)
                {
                    mout() << "Compensate Init Force A2" << std::endl;
                    mout()<<"Init 2 : "<<imp_->arm2_init_force[0]<<'\t'<<imp_->arm2_init_force[1]<<'\t'<<imp_->arm2_init_force[2]<<'\t'
                            <<imp_->arm2_init_force[3]<<'\t'<<imp_->arm2_init_force[4]<<'\t'<<imp_->arm2_init_force[5]<<std::endl;
                }
			}
            // 注意：对于 ModelComP 标定，始终使用原始数据
            // init_force 的减法移到 ForceDrag 等运行时使用的地方
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

		auto caculateAvgForce = [=](double arm1_force_data_[6], double arm2_force_data_[6], double arm1_temp_force_[6], double arm2_temp_force_[6], int count_)
		{
			if (count() < imp_->current_stop_time + imp_->stop_time)
			{

				if (count() % 200 == 0 && imp_->accumulation_count < 10)
				{
					double temp1[6]{ 0 };
					double temp2[6]{ 0 };

					getForceData(temp1, 0, imp_->init);
					getForceData(temp2, 1, imp_->init);

					for (int i = 0; i < 6; i++)
					{
						arm1_temp_force_[i] += temp1[i];
						arm2_temp_force_[i] += temp2[i];
					}


					imp_->accumulation_count = imp_->accumulation_count + 1;
					mout() << imp_->accumulation_count << std::endl;
				}

			}
			else if (count() == imp_->current_stop_time + imp_->stop_time)
			{
				mout() << "stop! " << "count(): " << count() << std::endl;
				imp_->accumulation_count = 0;
				for (int i = 0; i < 6; i++)
				{
					arm1_force_data_[i] = arm1_temp_force_[i] / 10.0;
					arm2_force_data_[i] = arm2_temp_force_[i] / 10.0;
				}
				mout() << "Arm 1 Force Data " << count_ << '\n' << arm1_force_data_[0] << '\t' << arm1_force_data_[1] << '\t' << arm1_force_data_[2] << '\t'
					<< arm1_force_data_[3] << '\t' << arm1_force_data_[4] << '\t' << arm1_force_data_[5] << std::endl;
				mout() << "Arm 2 Force Data " << count_ << '\n' << arm2_force_data_[0] << '\t' << arm2_force_data_[1] << '\t' << arm2_force_data_[2] << '\t'
					<< arm2_force_data_[3] << '\t' << arm2_force_data_[4] << '\t' << arm2_force_data_[5] << std::endl;
			}
			else
			{
				mout() << "Flag Change " << count_ << std::endl;
				imp_->stop_flag = false;
			}
		};


		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}


			if (imp_->stop_flag)
			{
				if (imp_->stop_count == 1)
				{
					caculateAvgForce(imp_->arm1_force_data_1, imp_->arm2_force_data_1, imp_->arm1_temp_force_1, imp_->arm2_temp_force_1, 1);
				}
				else if (imp_->stop_count == 2)
				{
					caculateAvgForce(imp_->arm1_force_data_2, imp_->arm2_force_data_2, imp_->arm1_temp_force_2, imp_->arm2_temp_force_2, 2);
				}
				else if (imp_->stop_count == 3)
				{
					caculateAvgForce(imp_->arm1_force_data_3, imp_->arm2_force_data_3, imp_->arm1_temp_force_3, imp_->arm2_temp_force_3, 3);
				}
				else if (imp_->stop_count == 4)
				{
					caculateAvgForce(imp_->arm1_force_data_4, imp_->arm2_force_data_4, imp_->arm1_temp_force_4, imp_->arm2_temp_force_4, 4);
				}
				else if (imp_->stop_count == 5)
				{
					caculateAvgForce(imp_->arm1_force_data_5, imp_->arm2_force_data_5, imp_->arm1_temp_force_5, imp_->arm2_temp_force_5, 5);
				}
				else
				{
					mout() << "Stop Count Wrong: " << imp_->stop_count << " stop flag: " << imp_->stop_flag << std::endl;
					return 0;
				}

				return 80000 - count();
			}
		else
		{
			static double init_angle[12] =
			{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, 0};

			static double angle1[12] =
			{  0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, PI /4};

			static double angle2[12] =
			{  0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 2, -PI /4};

			static double angle3[12] =
			{  0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, 5 * PI / 6, -7 * PI / 12, -PI / 4, 0};

			static double angle4[12] =
			{  0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, 5 * PI / 6, -7 * PI / 12, -3 * PI / 4, 0};

			static double angle5[12] =
			{  0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0};


			//// Arm 1 Angle
			//static double init_angle1[6]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 };

			//static double angle1_1[6]{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 };

			//static double angle1_2[6]{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0 };

			//static double angle1_3[6]{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 };




			//// Arm 2 Angle
			//static double init_angle2[6]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

			//static double angle2_1[6]{ 0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

			//static double angle2_2[6]{ 0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

			//static double angle2_3[6]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };

			if (!imp_->init)
			{

                dualArm.setInputPos(init_angle);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(init_angle);

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{

                    getForceData(imp_->arm1_init_force, 0, imp_->init);

					getForceData(imp_->arm2_init_force, 1, imp_->init);
					mout() << "Init Complete" << std::endl;

					imp_->init = true;
				}

			}

			if (!imp_->target1_reached && imp_->init)
			{
                dualArm.setInputPos(angle1);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle1);

				if (motorsPositionCheck(current_angle, angle1, 12))
				{
					mout() << "Target 1 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_1);
					eeA2.getMpm(imp_->arm2_ee_pm_1);

					imp_->target1_reached = true;
					imp_->stop_count = 1;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
                    //状态数据清零
                    for(int i=0;i<6;i++)
                    {
                        imp_->arm1_temp_force_1[i]=0; imp_->arm2_temp_force_1[i]=0;
                    }
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}

			}
			else if (imp_->target1_reached && !imp_->target2_reached)
			{
                dualArm.setInputPos(angle2);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle2);

				if (motorsPositionCheck(current_angle, angle2, 12))
				{
					mout() << "Target 2 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_2);
					eeA2.getMpm(imp_->arm2_ee_pm_2);

					imp_->target2_reached = true;
					imp_->stop_count = 2;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
                    //状态数据清零 (修复: 应该清零 temp_force_2)
                    for(int i=0;i<6;i++)
                    {
                        imp_->arm1_temp_force_2[i]=0; imp_->arm2_temp_force_2[i]=0;
                    }
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}
			}
			else if (imp_->target2_reached && !imp_->target3_reached)
			{
                dualArm.setInputPos(angle3);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle3);

				if (motorsPositionCheck(current_angle, angle3, 12))
				{
					mout() << "Target 3 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_3);
					eeA2.getMpm(imp_->arm2_ee_pm_3);

					imp_->target3_reached = true;
					imp_->stop_count = 3;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
                    //状态数据清零 (修复: 应该清零 temp_force_3)
                    for(int i=0;i<6;i++)
                    {
                        imp_->arm1_temp_force_3[i]=0; imp_->arm2_temp_force_3[i]=0;
                    }
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;

				}
			}
			else if (imp_->target3_reached && !imp_->target4_reached)
			{
                dualArm.setInputPos(angle4);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle4);

				if (motorsPositionCheck(current_angle, angle4, 12))
				{
					mout() << "Target 4 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_4);
					eeA2.getMpm(imp_->arm2_ee_pm_4);

					imp_->target4_reached = true;
					imp_->stop_count = 4;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
                    //状态数据清零
                    for(int i=0;i<6;i++)
                    {
                        imp_->arm1_temp_force_4[i]=0; imp_->arm2_temp_force_4[i]=0;
                    }
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}
			}
			else if (imp_->target4_reached && !imp_->target5_reached)
			{
                dualArm.setInputPos(angle5);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(angle5);

				if (motorsPositionCheck(current_angle, angle5, 12))
				{
					mout() << "Target 5 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_5);
					eeA2.getMpm(imp_->arm2_ee_pm_5);

					imp_->target5_reached = true;
					imp_->stop_count = 5;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
                    //状态数据清零
                    for(int i=0;i<6;i++)
                    {
                        imp_->arm1_temp_force_5[i]=0; imp_->arm2_temp_force_5[i]=0;
                    }
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}
			}
			else if (imp_->target5_reached && !imp_->target6_reached)
			{
				// Back To Init
                dualArm.setInputPos(init_angle);
                if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				daJointMove(init_angle);

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{
					mout() << "Back To Init Pos" << std::endl;
					imp_->target6_reached = true;

				}

			}
			else if (imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached && imp_->target5_reached && imp_->target6_reached)
			{
				// ========== Arm 1 标定（支持 5 个姿态） ==========
				const int num_poses = 5;
                double arm2_current_force[6]{0};
                double arm1_current_force[6]{ 0 };
				double arm1_ee_rm[5][9]{ 0 };
				double arm1_ee_rm_inv[5][9]{ 0 };
				double* arm1_force_data[5] = {
					imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3,
					imp_->arm1_force_data_4, imp_->arm1_force_data_5
				};
				double* arm1_ee_pm[5] = {
					imp_->arm1_ee_pm_1, imp_->arm1_ee_pm_2, imp_->arm1_ee_pm_3,
					imp_->arm1_ee_pm_4, imp_->arm1_ee_pm_5
				};

				// 提取旋转矩阵
				for (int i = 0; i < num_poses; ++i)
				{
					aris::dynamic::s_pm2rm(arm1_ee_pm[i], arm1_ee_rm[i]);
					gc.getInverseRm(arm1_ee_rm[i], arm1_ee_rm_inv[i]);
				}

				// 构建 R 矩阵（15行×6列）：R = [R1^T, I; R2^T, I; ...; R5^T, I]
				const int rows = num_poses * 3;  // 15行
				const int cols = 6;              // 6列
				double arm1_r_matrix[rows * cols]{ 0 };
				double arm1_f_vector[rows]{ 0 };

				for (int p = 0; p < num_poses; ++p)
				{
					// 每3行对应一个姿态
					int row_start = p * 3;
					// R^T 的前3列（旋转部分）
					arm1_r_matrix[(row_start + 0) * cols + 0] = arm1_ee_rm_inv[p][0];
					arm1_r_matrix[(row_start + 0) * cols + 1] = arm1_ee_rm_inv[p][3];
					arm1_r_matrix[(row_start + 0) * cols + 2] = arm1_ee_rm_inv[p][6];
					arm1_r_matrix[(row_start + 1) * cols + 0] = arm1_ee_rm_inv[p][1];
					arm1_r_matrix[(row_start + 1) * cols + 1] = arm1_ee_rm_inv[p][4];
					arm1_r_matrix[(row_start + 1) * cols + 2] = arm1_ee_rm_inv[p][7];
					arm1_r_matrix[(row_start + 2) * cols + 0] = arm1_ee_rm_inv[p][2];
					arm1_r_matrix[(row_start + 2) * cols + 1] = arm1_ee_rm_inv[p][5];
					arm1_r_matrix[(row_start + 2) * cols + 2] = arm1_ee_rm_inv[p][8];
					// 单位矩阵部分（零偏项）
					arm1_r_matrix[(row_start + 0) * cols + 3] = 1.0;
					arm1_r_matrix[(row_start + 1) * cols + 4] = 1.0;
					arm1_r_matrix[(row_start + 2) * cols + 5] = 1.0;
					// 力向量
					arm1_f_vector[row_start + 0] = arm1_force_data[p][0];
					arm1_f_vector[row_start + 1] = arm1_force_data[p][1];
					arm1_f_vector[row_start + 2] = arm1_force_data[p][2];
				}

				// Step 1: 求解 L 向量
				double U[rows * cols]{ 0 };
				double Inv_[rows * cols]{ 0 };
				double tau[rows]{ 0 };
				double tau2[rows]{ 0 };
				aris::Size p[rows];
				aris::Size rank;
				aris::dynamic::s_householder_utp(rows, cols, arm1_r_matrix, U, tau, p, rank, 1e-6);
				aris::dynamic::s_householder_utp2pinv(rows, cols, rank, U, tau, p, Inv_, tau2, 1e-6);
				aris::dynamic::s_mm(cols, 1, rows, Inv_, arm1_f_vector, imp_->arm1_l_vector);

				// Step 2: 计算每个姿态下的真实重力 G = R^{-1} × L[0:3]
				double arm1_L_vec[3] = { imp_->arm1_l_vector[0], imp_->arm1_l_vector[1], imp_->arm1_l_vector[2] };
				double arm1_G[5][3]{ 0 };
				for (int p = 0; p < num_poses; ++p)
				{
					aris::dynamic::s_mm(3, 1, 3, arm1_ee_rm_inv[p], arm1_L_vec, arm1_G[p]);
				}

				// Step 3: 构建 F 矩阵（15行×6列），用真实重力
				double arm1_f_matrix[rows * cols]{ 0 };
				double arm1_t_vector[rows]{ 0 };

				for (int p = 0; p < num_poses; ++p)
				{
					int row_start = p * 3;
					// F 矩阵：叉乘矩阵 [0, Gz, -Gy, 1, 0, 0; -Gz, 0, Gx, 0, 1, 0; Gy, -Gx, 0, 0, 0, 1]
					double Gx = arm1_G[p][0], Gy = arm1_G[p][1], Gz = arm1_G[p][2];
					arm1_f_matrix[(row_start + 0) * cols + 0] = 0;
					arm1_f_matrix[(row_start + 0) * cols + 1] = Gz;
					arm1_f_matrix[(row_start + 0) * cols + 2] = -Gy;
					arm1_f_matrix[(row_start + 0) * cols + 3] = 1.0;
					arm1_f_matrix[(row_start + 1) * cols + 0] = -Gz;
					arm1_f_matrix[(row_start + 1) * cols + 1] = 0;
					arm1_f_matrix[(row_start + 1) * cols + 2] = Gx;
					arm1_f_matrix[(row_start + 1) * cols + 4] = 1.0;
					arm1_f_matrix[(row_start + 2) * cols + 0] = Gy;
					arm1_f_matrix[(row_start + 2) * cols + 1] = -Gx;
					arm1_f_matrix[(row_start + 2) * cols + 2] = 0;
					arm1_f_matrix[(row_start + 2) * cols + 5] = 1.0;
					// 力矩向量
					arm1_t_vector[row_start + 0] = arm1_force_data[p][3];
					arm1_t_vector[row_start + 1] = arm1_force_data[p][4];
					arm1_t_vector[row_start + 2] = arm1_force_data[p][5];
				}

				// Step 4: 求解 P 向量
				aris::dynamic::s_householder_utp(rows, cols, arm1_f_matrix, U, tau, p, rank, 1e-6);
				aris::dynamic::s_householder_utp2pinv(rows, cols, rank, U, tau, p, Inv_, tau2, 1e-6);
				aris::dynamic::s_mm(cols, 1, rows, Inv_, arm1_t_vector, imp_->arm1_p_vector);

				double arm1_current_ee_pm[16]{ 0 };
				double arm1_compf[6]{ 0 };
				eeA1.getMpm(arm1_current_ee_pm);
				gc.getCompFT(arm1_current_ee_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_compf);
				getForceData(arm1_current_force, 0, imp_->init);

				// ========== Arm 2 标定（支持 5 个姿态） ==========
				double arm2_ee_rm[5][9]{ 0 };
				double arm2_ee_rm_inv[5][9]{ 0 };
				double* arm2_force_data[5] = {
					imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3,
					imp_->arm2_force_data_4, imp_->arm2_force_data_5
				};
				double* arm2_ee_pm[5] = {
					imp_->arm2_ee_pm_1, imp_->arm2_ee_pm_2, imp_->arm2_ee_pm_3,
					imp_->arm2_ee_pm_4, imp_->arm2_ee_pm_5
				};

				// 提取旋转矩阵
				for (int i = 0; i < num_poses; ++i)
				{
					aris::dynamic::s_pm2rm(arm2_ee_pm[i], arm2_ee_rm[i]);
					gc.getInverseRm(arm2_ee_rm[i], arm2_ee_rm_inv[i]);
				}

				// 构建 R 矩阵（15行×6列）
				double arm2_r_matrix[rows * cols]{ 0 };
				double arm2_f_vector[rows]{ 0 };

				for (int p = 0; p < num_poses; ++p)
				{
					int row_start = p * 3;
					// R^T 的前3列（旋转部分）
					arm2_r_matrix[(row_start + 0) * cols + 0] = arm2_ee_rm_inv[p][0];
					arm2_r_matrix[(row_start + 0) * cols + 1] = arm2_ee_rm_inv[p][3];
					arm2_r_matrix[(row_start + 0) * cols + 2] = arm2_ee_rm_inv[p][6];
					arm2_r_matrix[(row_start + 1) * cols + 0] = arm2_ee_rm_inv[p][1];
					arm2_r_matrix[(row_start + 1) * cols + 1] = arm2_ee_rm_inv[p][4];
					arm2_r_matrix[(row_start + 1) * cols + 2] = arm2_ee_rm_inv[p][7];
					arm2_r_matrix[(row_start + 2) * cols + 0] = arm2_ee_rm_inv[p][2];
					arm2_r_matrix[(row_start + 2) * cols + 1] = arm2_ee_rm_inv[p][5];
					arm2_r_matrix[(row_start + 2) * cols + 2] = arm2_ee_rm_inv[p][8];
					// 单位矩阵部分（零偏项）
					arm2_r_matrix[(row_start + 0) * cols + 3] = 1.0;
					arm2_r_matrix[(row_start + 1) * cols + 4] = 1.0;
					arm2_r_matrix[(row_start + 2) * cols + 5] = 1.0;
					// 力向量
					arm2_f_vector[row_start + 0] = arm2_force_data[p][0];
					arm2_f_vector[row_start + 1] = arm2_force_data[p][1];
					arm2_f_vector[row_start + 2] = arm2_force_data[p][2];
				}

				// Step 1: 求解 L 向量
				double U2[rows * cols]{ 0 };
				double Inv2_[rows * cols]{ 0 };
				double tau2_[rows]{ 0 };
				double tau22[rows]{ 0 };
				aris::Size p2[rows];
				aris::Size rank2;
				aris::dynamic::s_householder_utp(rows, cols, arm2_r_matrix, U2, tau2_, p2, rank2, 1e-6);
				aris::dynamic::s_householder_utp2pinv(rows, cols, rank2, U2, tau2_, p2, Inv2_, tau22, 1e-6);
				aris::dynamic::s_mm(cols, 1, rows, Inv2_, arm2_f_vector, imp_->arm2_l_vector);

				// Step 2: 计算每个姿态下的真实重力 G = R^{-1} × L[0:3]
				double arm2_L_vec[3] = { imp_->arm2_l_vector[0], imp_->arm2_l_vector[1], imp_->arm2_l_vector[2] };
				double arm2_G[5][3]{ 0 };
				for (int p = 0; p < num_poses; ++p)
				{
					aris::dynamic::s_mm(3, 1, 3, arm2_ee_rm_inv[p], arm2_L_vec, arm2_G[p]);
				}

				// Step 3: 构建 F 矩阵（15行×6列），用真实重力
				double arm2_f_matrix[rows * cols]{ 0 };
				double arm2_t_vector[rows]{ 0 };

				for (int p = 0; p < num_poses; ++p)
				{
					int row_start = p * 3;
					// F 矩阵：叉乘矩阵
					double Gx = arm2_G[p][0], Gy = arm2_G[p][1], Gz = arm2_G[p][2];
					arm2_f_matrix[(row_start + 0) * cols + 0] = 0;
					arm2_f_matrix[(row_start + 0) * cols + 1] = Gz;
					arm2_f_matrix[(row_start + 0) * cols + 2] = -Gy;
					arm2_f_matrix[(row_start + 0) * cols + 3] = 1.0;
					arm2_f_matrix[(row_start + 1) * cols + 0] = -Gz;
					arm2_f_matrix[(row_start + 1) * cols + 1] = 0;
					arm2_f_matrix[(row_start + 1) * cols + 2] = Gx;
					arm2_f_matrix[(row_start + 1) * cols + 4] = 1.0;
					arm2_f_matrix[(row_start + 2) * cols + 0] = Gy;
					arm2_f_matrix[(row_start + 2) * cols + 1] = -Gx;
					arm2_f_matrix[(row_start + 2) * cols + 2] = 0;
					arm2_f_matrix[(row_start + 2) * cols + 5] = 1.0;
					// 力矩向量
					arm2_t_vector[row_start + 0] = arm2_force_data[p][3];
					arm2_t_vector[row_start + 1] = arm2_force_data[p][4];
					arm2_t_vector[row_start + 2] = arm2_force_data[p][5];
				}

				// Step 4: 求解 P 向量
				aris::dynamic::s_householder_utp(rows, cols, arm2_f_matrix, U2, tau2_, p2, rank2, 1e-6);
				aris::dynamic::s_householder_utp2pinv(rows, cols, rank2, U2, tau2_, p2, Inv2_, tau22, 1e-6);
				aris::dynamic::s_mm(cols, 1, rows, Inv2_, arm2_t_vector, imp_->arm2_p_vector);

				double arm2_current_ee_pm[16]{ 0 };
                double arm2_compf[6]{0};


				eeA2.getMpm(arm2_current_ee_pm);

                gc.getCompFT(arm2_current_ee_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_compf);

                getForceData(arm2_current_force, 1, imp_->init);

                gc.savePLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);


                mout() << "Current Arm1 Force:" << '\n' << arm1_current_force[0] << '\t' << arm1_current_force[1] << '\t'
                    << arm1_current_force[2] << '\t' << arm1_current_force[3] << '\t'
                    << arm1_current_force[4] << '\t' << arm1_current_force[5] << std::endl;

                mout() << "Current Arm2 Force:" << '\n' << arm2_current_force[0]  << '\t' << arm2_current_force[1]  << '\t'
                    << arm2_current_force[2] << '\t' << arm2_current_force[3]  << '\t'
                    << arm2_current_force[4] << '\t' << arm2_current_force[5] << std::endl;



                mout() << "Current Arm1 Force After Compensation:" << '\n' << arm1_current_force[0] + arm1_compf[0] << '\t' << arm1_current_force[1] + arm1_compf[1] << '\t'
                    << arm1_current_force[2] + arm1_compf[2] << '\t' << arm1_current_force[3] + arm1_compf[3] << '\t'
                    << arm1_current_force[4] + arm1_compf[4] << '\t' << arm1_current_force[5] + arm1_compf[5] << std::endl;

                mout() << "Current Arm2 Force After Compensation:" << '\n' << arm2_current_force[0] + arm2_compf[0] << '\t' << arm2_current_force[1] + arm2_compf[1] << '\t'
                    << arm2_current_force[2] + arm2_compf[2] << '\t' << arm2_current_force[3] + arm2_compf[3] << '\t'
                    << arm2_current_force[4] + arm2_compf[4] << '\t' << arm2_current_force[5] + arm2_compf[5] << std::endl;




                return 0;

			}

			if (count() == 100000)
			{


				mout() << "Over Time" << std::endl;


			}

			return 100000 - count();

		}
	}
	ModelComP::ModelComP(const std::string& name) : imp_(new Imp)
	{

		aris::core::fromXmlString(command(),
             "<Command name=\"m_comp\"/>");
	}
	ModelComP::~ModelComP() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ModelComP)



	struct ForceAlign::Imp {

		bool init = false;
		bool contact_check = false;

		double comp_f[6]{ 0 };

		double arm1_init_force[6]{ 0 };
		double arm2_init_force[6]{ 0 };

		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		double x_d;

		double Ke = 220000;
		double K = 3;

		double B[6]{ 0.25,0.25,0.7,0.0,0.0,0.0 };
		double M[6]{ 0.1,0.1,0.1,0.1,0.1,0.1 };

		double desired_force = -5;
		int contact_count;

		//Switch Model
		int m_;

	};
	auto ForceAlign::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;

	}
	auto ForceAlign::executeRT()->int
	{
        if (!require_dual_arm(*this, "m_fa"))
        {
            return 0;
        }

		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

        static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
        static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
        static double max_force[6]{ 10,10,10,5,5,5 };
        static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		imp_->m_ = int32Param("model");

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



		double current_vel[6]{ 0 };
		double current_pos[6]{ 0 };

		double current_force[6]{ 0 };
		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };
		double current_pm[16]{ 0 };

		double comp_force[6]{ 0 };
		double actual_force[6]{ 0 };

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

			}

			if (m_ == 0)
			{
				//				data_[0] = -data_[0];
				//				data_[1] = -data_[1];

				//				data_[3] = -data_[3];
				//				data_[4] = -data_[4];
			}
			else if (m_ == 1)
			{
				data_[0] = -data_[0];
				data_[1] = -data_[1];

				data_[3] = -data_[3];
				data_[4] = -data_[4];

			}
			else
			{
				mout() << "Wrong Model" << std::endl;
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

		//single arm move 1-->white 2-->blue
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


		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if (dualArm.forwardKinematics())
			{
				mout() << "forward fail" << std::endl;
			}

			daJointMove(init_angle);
			if (motorsPositionCheck(current_angle, init_angle, 12))
			{
				getForceData(imp_->arm1_init_force, 0, imp_->init);
				getForceData(imp_->arm2_init_force, 1, imp_->init);
				mout() << "Back To Init" << std::endl;

				imp_->init = true;
			}
		}
		else
		{
			//Arm1
			if (imp_->m_ == 0)
			{
				//Update Statue
				eeA1.getV(current_vel);
				eeA1.getP(current_pos);
				eeA1.getMpm(current_pm);

				std::copy(current_angle, current_angle + 6, current_sa_angle);

				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}



				//Contact Check



				//Get Actual Force
				getForceData(current_force, imp_->m_, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				if (!imp_->contact_check)
				{
					if (abs(actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a1, 0);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a1, 0);

					if (count() % 100 == 0)
					{
						//mout() << "count(): " << count() << std::endl;
						mout() << "force: " << actual_force[0] << '\t' << actual_force[1] << '\t' << actual_force[2] << '\t'
							<< actual_force[3] << '\t' << actual_force[4] << '\t' << actual_force[5] << std::endl;

					}

				}
			}
			//Arm2
			else if (imp_->m_ == 1)
			{
				//Update Statue
				eeA2.getV(current_vel);
				eeA2.getP(current_pos);
				eeA2.getMpm(current_pm);

				std::copy(current_angle + 6, current_angle + 12, current_sa_angle);


				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}


				//Contact Check

                //Get Actual Force
				getForceData(current_force, imp_->m_, imp_->init);
				gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				if (!imp_->contact_check)
				{
					if (abs(actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a2, 1);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a2, 1);

					if (count() % 1000 == 0)
					{
						//mout() << "count(): " << count() << std::endl;
                        mout() << "force: " << actual_force[0] << '\t' << actual_force[1] << '\t' << actual_force[2] << '\t'
                            << actual_force[3] << '\t' << actual_force[4] << '\t' << actual_force[5] << std::endl;

					}

				}
			}
			//Error
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}

		}
		//Over Time Exit
		if (count() == 800000)
		{
			mout() << "Over Time" << std::endl;
		}

		return 800000 - count();

	}
	ForceAlign::ForceAlign(const std::string& name) : imp_(new Imp)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_fa\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceAlign::~ForceAlign() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ForceAlign)


	struct ForceKeep::Imp {

		//Flag
		bool init = false;
		bool contact_check = false;

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

		//Current Vel
		double v_c[6]{ 0 };

		//Impedence Parameter
		double a1_K[6]{ 100,100,100,5,5,5 };
		double a1_B[6]{ 100,100,100,5,5,5 };
		double a1_M[6]{ 1,1,1,2,2,2 };

		// double a2_K[6]{ 100,100,100,15,15,15 };
		// double a2_B[6]{ 100,100,100,15,15,15 };
		// double a2_M[6]{ 1,1,1,10,10,10 };

		// double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

        //手感更软
        double a2_K[6]{ 0.2,0.2,0.2,1,1,1};
		double a2_B[6]{ 3,3,3,1,1,1  };
		double a2_M[6]{ 0.5,0.5,0.5,2,2,2 };

		double Ke[6]{ 22000,22000,22000,22000,22000,22000 };
		//Counter
        int contact_count = 0;

		//Switch Model
		int m_;

        //Force Buffer
        std::array<double, 10> force_buffer[6] = {};
        int buffer_index[6]{ 0 };
	};
	auto ForceKeep::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceKeep::executeRT() -> int
	{
        if (!require_dual_arm(*this, "m_fk"))
        {
            return 0;
        }
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
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.5,0.5,0.5,0.005,0.005,0.001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 15,15,15,10,10,10 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double current_force[6]{ 0 };
        double actual_force[6]{ 0 };
        double filtered_force[6]{0};
        double transform_force[6]{0};

		imp_->m_ = int32Param("model");

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
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
            }
        };


		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



        if (!imp_->init && !imp_->contact_check)
		{



			dualArm.setInputPos(init_angle);

			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}


			daJointMove(init_angle);

			if (count() % 1000 == 0)
			{

				mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
					<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

			}

			if (motorsPositionCheck(current_angle, init_angle, 12))
			{

				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);

				getForceData(imp_->arm1_init_force, 0, imp_->init);
				getForceData(imp_->arm2_init_force, 1, imp_->init);

				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}


		}

        else if (imp_->init && !imp_->contact_check)
        {
            double raw_force_checker[12]{ 0 };
            double comp_force_checker[12]{ 0 };
            double force_checker[12]{ 0 };

            double a1_pm[16]{ 0 };
            double a2_pm[16]{ 0 };

            eeA1.getMpm(a1_pm);
            eeA2.getMpm(a2_pm);


            //Arm1
            getForceData(raw_force_checker, 0, imp_->init);
            gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
            //Arm2
            getForceData(raw_force_checker + 6, 1, imp_->init);
            gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);


            for (int i = 0; i < 12; i++)
            {
                force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

                if (abs(force_checker[i]) > 3.0)
                {
                    imp_->contact_check = true;
                    mout() << "Contact Check" << std::endl;
                    break;
                }
            }
        }
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };
				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				getForceData(current_force, 0, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					// getForceData 在 init==true 时返回 (raw - init_force)，这里恢复成 raw 再做重力补偿
					double raw_i = current_force[i] + imp_->arm1_init_force[i];
					actual_force[i] = comp_force[i] + raw_i;
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

                //Force Filter
                forceFilter(actual_force,filtered_force);
                // //Coordinate Transform Arm1
                transform_force[0] = filtered_force[2];
                transform_force[1] = -filtered_force[1];
                transform_force[2] = filtered_force[0];

                transform_force[3] = filtered_force[5];
                transform_force[4] = -filtered_force[4];
                transform_force[5] = filtered_force[3];



				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
                double dt = 0.002;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                    acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->a1_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a1_K[i] * (current_pos[i] - imp_->arm1_x_d[i])) / imp_->a1_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}

				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
                double inv_rm_d[9]{0};
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm1_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
                //aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);
                gc.getInverseRm(rm_d, inv_rm_d);
                aris::dynamic::s_mm(3,3,3,rm_c,inv_rm_d,rm_e);

				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);



				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
                    ome[i] = (-imp_->f_d[i + 3] + transform_force[i + 3] - imp_->a1_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a1_K[i + 3] * pose_error[i]) / imp_->a1_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");


				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a1, 0);




				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					 mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					 	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

					//mout() << "error: " << pose_error[0] << '\t' << pose_error[1] << '\t' << pose_error[2] << std::endl;

				}

			}
			else if (imp_->m_ == 1)
			{

				double current_vel[6]{ 0 };

				eeA2.getP(current_pos);

				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);


                //都是在末端坐标系下的力
				getForceData(current_force, 1, imp_->init);
				gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}



				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

                //Coordinate Transform Arm2
                transform_force[0] = -filtered_force[2];
                transform_force[1] = filtered_force[1];
                transform_force[2] = filtered_force[0];

                transform_force[3] = -filtered_force[5];
                transform_force[4] = filtered_force[4];
                transform_force[5] = filtered_force[3];


				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
                double dt = 0.002;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M

                    acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->a2_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a2_K[i] * (current_pos[i] - imp_->arm2_x_d[i])) / imp_->a2_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
                double inv_rm_d[9]{0};
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm2_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
                //aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);
                gc.getInverseRm(rm_d, inv_rm_d);
                aris::dynamic::s_mm(3,3,3,rm_c,inv_rm_d,rm_e);


				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);




				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
                    ome[i] = (-imp_->f_d[i + 3] + transform_force[i + 3] - imp_->a2_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a2_K[i + 3] * pose_error[i]) / imp_->a2_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

				eeA2.setV(imp_->v_c);
				if (model_a2.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a2, 1);



				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}





		}

        return 30000 - count();
	}
	ForceKeep::ForceKeep(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fk\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceKeep::~ForceKeep() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ForceKeep)



	struct ForceDrag::Imp {

		//Flag
		bool init = false;
		bool contact_check = false;

        //舵机控制
        bool gripper_flag =false;
        double gripper_last_state =1;

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

        //从臂运动目标角度，从tcp更新这个参数
        double target_q[6]{0};

		double v_d[6]{ 0 };
		double a_d[6]{ 0 };
		double f_d[6]{ 0 };

		//Current Vel
		double v_c[6]{ 0 };

		//Impedence Parameter
        // 拖动示教：平衡灵敏度和稳定性
		double K[6]{ 0, 0, 0, 0, 0, 0 };
        
        // 阻尼：适中
        double B[6]{ 150, 150, 150, 4, 4, 4 };
        
        // 惯性：增大，降低响应速度
        double M[6]{ 13, 13, 13, 1.0, 1.0, 1.0 };

		double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

        // 力增益：降低，减少噪声放大
        double gain_trans = 1.8;
        double gain_rot = 3.0;

        //Parameters For Compensating rz
        // double a_y = -0.0592;
        // double b_y = -0.9943;
        // double c_y = 0.0132;


		//Counter
		int contact_count = 0;

		//Test
		double actual_force[6]{ 0 };

		//Switch Model
		int m_;

        //Force Buffer - 增大窗口减少噪声
        std::array<double, 30> force_buffer[6] = {};
        int buffer_index[6]{ 0 };
	};
	auto ForceDrag::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceDrag::executeRT() -> int
	{
        if (!require_dual_arm(*this, "m_fd"))
        {
            return 0;
        }
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
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		static double max_vel[6]{ 0.5,0.5,0.5,0.005,0.005,0.001 };

        //static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
        static double trigger_force[6]{ 0.2,0.2,0.2,0.02,0.02,0.02 };
		static double max_force[6]{ 5,5,5,1,1,1};
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double current_force[6]{ 0 };
        double actual_force[6]{ 0 };
        double filtered_force[6]{0};
        double transform_force[6]{0};

        double rz_comp = 0;
        double rz_comp_force[6]{0};

		imp_->m_ = int32Param("model");

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
                //std::cout<<"arm2 控制角度：";
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
                    //std::cout<< x_joint[i]<<" ,";
				}
                //std::cout<<std::endl;
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
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 30;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 30.0;
            }
        };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}

        if (!imp_->init && !imp_->contact_check)
		{



//			dualArm.setInputPos(init_angle);

//			if (dualArm.forwardKinematics())
//			{
//				throw std::runtime_error("Forward Kinematics Position Failed!");
//			}



//			daJointMove(init_angle);

//			if (count() % 1000 == 0)
//			{

//				mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
//					<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

//			}

//			if (motorsPositionCheck(current_angle, init_angle, 12))
//			{

//				eeA1.getP(imp_->arm1_x_d);
//				eeA2.getP(imp_->arm2_x_d);

//				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
//					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;

//				getForceData(imp_->arm1_init_force, 0, imp_->init);
//				getForceData(imp_->arm2_init_force, 1, imp_->init);

//				mout() << "Back To Init" << std::endl;
//				imp_->init = true;
//			}

            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);
            //master()->logFileRawName(std::string("/home/kaanh/Desktop/kaanhbin/force_comp_data/forceComp2_" + aris::core::logFileTimeFormat(std::chrono::system_clock::now())).c_str());
            
            // 初始化期望位置为当前位置（防止漂移）
            eeA1.getP(imp_->arm1_x_d);
            eeA2.getP(imp_->arm2_x_d);
            
            imp_->init = true;

		}
        else if (imp_->init && !imp_->contact_check)
        {
            double raw_force_checker[12]{ 0 };
            double comp_force_checker[12]{ 0 };
            double force_checker[12]{ 0 };

            double a1_pm[16]{ 0 };
            double a2_pm[16]{ 0 };

            eeA1.getMpm(a1_pm);
            eeA2.getMpm(a2_pm);


            //Arm1
            getForceData(raw_force_checker, 0, imp_->init);
            gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
            //Arm2
            getForceData(raw_force_checker + 6, 1, imp_->init);
            gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);

            // getForceData 在 init==true 时返回 (raw - init_force)，这里恢复成 raw
            for (int i = 0; i < 6; ++i)
            {
                raw_force_checker[i]     += imp_->arm1_init_force[i];
                raw_force_checker[i + 6] += imp_->arm2_init_force[i];
            }

                    for (int i = 0; i < 12; i++) 
                    {
                        force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

                        if (!std::isfinite(force_checker[i])) {
                            mout() << "[BAD] force_checker NaN/Inf i=" << i << std::endl;
                            return 0;
                        }

                        if (std::abs(force_checker[i]) > 3.0) {
                            imp_->contact_check = true;
                            break;
                        }
                    }
        }
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };

				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				getForceData(current_force, 0, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

                //Force Filter
                forceFilter(actual_force,filtered_force);

                //Coordinate Transform Arm1
                transform_force[0] = filtered_force[2];
                transform_force[1] = -filtered_force[1];
                transform_force[2] = filtered_force[0];

                transform_force[3] = filtered_force[5];
                transform_force[4] = -filtered_force[4];
                transform_force[5] = filtered_force[3];



				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
                double dt = 0.002;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                    acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
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
                    ome[i] = (-imp_->f_d[i + 3] + transform_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
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

				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
                //saMove(current_pos, model_a1, 0);
                
                // //------------ 3. 从臂：通过 TCP 跟随 Leader 的目标关节 ------------
                // m==0
                // if (g_tcp_server)
                // {
                //     std::vector<double> target_q = g_tcp_server->get_target_q();


                //     // static int printcount = 0;
                //     // if( printcount++ % 100 ==0)
                //     // {   std::cout << "点击控制q向量:[";
                //     //     for ( int i = 0; i < target_q.size(); ++i)
                //     //     {
                //     //         std::cout << target_q[i] << (i < target_q.size() - 1 ? ",":"" );
                //     //     }
                //     //     std::cout << "]\n";
                //     // }

                //     // 这里假设 target_q 是从 Leader 端发来的 6 关节角（或者你自己约定的格式）
                //     if (target_q.size() >= 6)
                //     {
                //         for (int i = 0; i < 6; ++i)
                //         {
                //             // 从臂电机索引 6~11
                //             //添加负号，让主从臂对称运动
                //             controller()->motorPool()[i + 6].setTargetPos(target_q[i]);
                //         }
                //     }
                // }


				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

                    mout() <<"Pos: "<<'\t'<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] <<'\t'
                          <<"Force: "<< current_force[0] << '\t' << current_force[1] << '\t' << current_force[2] <<'\t'
                         << current_force[3] << '\t' << current_force[4] << '\t' << current_force[5] <<'\t'<< std::endl;

				}

			}
            // ======= ForceDrag: m == 1 branch (Arm2 导纳力控拖动) =======
            else if (imp_->m_ == 1)
            {
                // ----------------- 0. 安全：关键指针 / NaN 检查 -----------------
                if (!imp_->init)
                {
                    mout() << "[WARN] ForceDrag called but imp_->init == false" << std::endl;
                    return 30000 - count();
                }

                // ----------------- 1. 读取当前末端状态 -----------------
                double current_vel[6]{0};
                eeA2.getP(current_pos);          // pe: [x,y,z,rx,ry,rz] in base frame
                eeA2.getV(current_vel);          // ee cartesian velocity
                eeA2.getMpm(current_pm);         // base_T_ee 4x4 pm
                
                // 首次进入时，初始化期望位置为当前位置
                static bool first_run = true;
                if (first_run)
                {
                    std::copy(current_pos, current_pos + 6, imp_->arm2_x_d);
                    first_run = false;
                    mout() << "[INFO] Initialize arm2_x_d to current position" << std::endl;
                }

                // 从 current_pm 里取出 R_be (base <- ee 的旋转矩阵, 用于 ee->base: F_b = R_be * F_e)
                double rm_be[9];
                rm_be[0] = current_pm[0];   rm_be[1] = current_pm[1];   rm_be[2] = current_pm[2];
                rm_be[3] = current_pm[4];   rm_be[4] = current_pm[5];   rm_be[5] = current_pm[6];
                rm_be[6] = current_pm[8];   rm_be[7] = current_pm[9];   rm_be[8] = current_pm[10];

                // ----------------- 2. 力传感器 + 重力补偿（EE系） -----------------
                double current_force[6]{0};    // sensor/ee frame, init removed inside getForceData
                double comp_force[6]{0};       // grav-comp in ee frame (from PL)
                double actual_force[6]{0};     // contact = comp + sensor
                double filtered_force[6]{0};   // filtered contact force (ee frame)

                getForceData(current_force, 1, imp_->init);
                gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);


                for (int i = 0; i < 6; ++i)
                {
                    actual_force[i] = comp_force[i] + current_force[i];
                    if (!std::isfinite(actual_force[i]))
                    {
                        mout() << "[BAD] actual_force NaN/Inf i=" << i
                            << " cur=" << current_force[i]
                            << " comp=" << comp_force[i] << std::endl;
                        return 0;
                    }
                }

                if (count() % 50 == 0)
                {
                    lout() << "raw\t"
                        << current_force[0] + imp_->arm2_init_force[0] << '\t'
                        << current_force[1] + imp_->arm2_init_force[1] << '\t'
                        << current_force[2] + imp_->arm2_init_force[2] << '\t'
                        << current_force[3] + imp_->arm2_init_force[3] << '\t'
                        << current_force[4] + imp_->arm2_init_force[4] << '\t'
                        << current_force[5] + imp_->arm2_init_force[5] << '\t'
                        << "comp+raw\t"
                        << actual_force[0] << '\t'
                        << actual_force[1] << '\t'
                        << actual_force[2] << '\t'
                        << actual_force[3] << '\t'
                        << actual_force[4] << '\t'
                        << actual_force[5] << std::endl;
                }

                // 滤波（在 EE/传感器系）
                forceFilter(actual_force, filtered_force);

                // ----------------- 3. 力从 EE 坐标系变到 Base 坐标系 -----------------
                double F_e[3]   = { filtered_force[0], filtered_force[1], filtered_force[2] };
                double Tau_e[3] = { filtered_force[3], filtered_force[4], filtered_force[5] };

                double F_b[3]{0};
                double Tau_b[3]{0};

                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        F_b[i]   += rm_be[3 * i + j] * F_e[j];
                        Tau_b[i] += rm_be[3 * i + j] * Tau_e[j];
                    }
                    if (!std::isfinite(F_b[i]) || !std::isfinite(Tau_b[i]))
                    {
                        mout() << "[BAD] transform to base NaN/Inf i=" << i << std::endl;
                        return 0;
                    }
                }

                double transform_force[6]{0};
                transform_force[0] = F_b[0]   * imp_->gain_trans;
                transform_force[1] = F_b[1]   * imp_->gain_trans;
                transform_force[2] = F_b[2]   * imp_->gain_trans;
                transform_force[3] = Tau_b[0] * imp_->gain_rot;
                transform_force[4] = Tau_b[1] * imp_->gain_rot;
                transform_force[5] = Tau_b[2] * imp_->gain_rot;

                // ----------------- 3.5 零偏估计（仅初始阶段，之后固定） -----------------
                static double bias[6] = {0};
                static int bias_init_count = 0;
                const int bias_init_samples = 200;  // 仅前200个周期估计零偏
                
                // 仅在初始阶段估计零偏，之后固定不变
                if (bias_init_count < bias_init_samples)
                {
                    const double alpha = 0.05;  // 较快学习
                    for (int i = 0; i < 6; ++i)
                        bias[i] = (1.0 - alpha) * bias[i] + alpha * transform_force[i];
                    bias_init_count++;
                }
                
                // 应用零偏补偿
                for (int i = 0; i < 6; ++i)
                    transform_force[i] -= bias[i];

                // if (count() % 500 == 0)
                // {
                //     mout() << "Force_b:\t"
                //         << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                //         << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
                // }

                // ----------------- 4. 死区 + 饱和 -----------------
                // PL=0时，需要更大的死区来过滤重力影响（约3N）
                double trigger_force[6]{ 3.5, 3.5, 4.0, 0.25, 0.25, 0.25 };
                double max_force[6]{ 30, 30, 30, 4, 4, 4 };  // 限制最大力，避免过激响应

                for (int i = 0; i < 6; ++i)
                {
                    if (std::abs(transform_force[i]) < trigger_force[i])
                        transform_force[i] = 0.0;

                    if (transform_force[i] >  max_force[i]) transform_force[i] =  max_force[i];
                    if (transform_force[i] < -max_force[i]) transform_force[i] = -max_force[i];
                }

                // ----------------- 5. 6 维导纳控制（拖动示教，K=0） -----------------
                double acc[3]{0};
                double ome[3]{0};
                double dx[3]{0};
                double dth[3]{0};
                const double dt = 0.002;

                // 5.1 平移：简化的导纳控制 acc = (F - B*v) / M
                for (int i = 0; i < 3; ++i)
                {
                    // 拖动示教：无位置反馈（K=0），只有阻尼
                    acc[i] = (transform_force[i] - imp_->B[i] * imp_->v_c[i]) / imp_->M[i];

                    if (!std::isfinite(acc[i]))
                    {
                        mout() << "[BAD] acc NaN/Inf i=" << i << std::endl;
                        return 0;
                    }
                }

                for (int i = 0; i < 3; ++i)
                {
                    imp_->v_c[i] += acc[i] * dt;
                    
                    // 限制速度
                    const double max_v = 0.15;  // m/s，降低最大速度
                    if (std::abs(imp_->v_c[i]) > max_v)
                        imp_->v_c[i] = (imp_->v_c[i] > 0 ? max_v : -max_v);
                    
                    // 小速度清零（防止漂移）
                    if (std::abs(imp_->v_c[i]) < 0.001)
                        imp_->v_c[i] = 0.0;
                    
                    dx[i] = imp_->v_c[i] * dt;
                    current_pos[i] += dx[i];
                }

                // 5.2 旋转：简化的导纳控制
                double rm_c[9]{0};
                
                for (int i = 0; i < 3; ++i)
                {
                    // 拖动示教：无姿态反馈（K=0），只有阻尼
                    ome[i] = (transform_force[i + 3] - imp_->B[i + 3] * imp_->v_c[i + 3]) / imp_->M[i + 3];

                    if (!std::isfinite(ome[i]))
                    {
                        mout() << "[BAD] ome NaN/Inf i=" << i << std::endl;
                        return 0;
                    }
                }

                for (int i = 0; i < 3; ++i)
                {
                    imp_->v_c[i + 3] += ome[i] * dt;
                    
                    // 限制角速度
                    const double max_omega = 0.3;  // rad/s
                    if (std::abs(imp_->v_c[i + 3]) > max_omega)
                        imp_->v_c[i + 3] = (imp_->v_c[i + 3] > 0 ? max_omega : -max_omega);
                    
                    // 小角速度清零（防止漂移）
                    if (std::abs(imp_->v_c[i + 3]) < 0.001)
                        imp_->v_c[i + 3] = 0.0;
                    
                    dth[i] = imp_->v_c[i + 3] * dt;
                }

                // 小角度 dth -> 增量旋转矩阵，再左乘当前姿态
                double drm[9]{0};
                double rm_target[9]{0};

                aris::dynamic::s_ra2rm(dth, drm);
                aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");
                aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);
                aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                // ----------------- 6. Arm2：速度+位置下发 -----------------
                eeA2.setV(imp_->v_c);

                if (model_a2.inverseKinematicsVel())
                {
                    mout() << "[ERROR] inverseKinematicsVel failed (arm2)" << std::endl;
                }
                else
                {
                    saMove(current_pos, model_a2, 1);  // mode=1 表示 arm2
                }

                // ----------------- 6. 下发控制（你现在用 TCP 控 follower） -----------------
                // ----------------- follower via TCP (30Hz target, 500Hz smooth) -----------------

                // if (g_tcp_server)
                // {   
                //     // 1) 读取一次TCP目标（可能30Hz更新，但我们500Hz都读也没事）
                //     auto target = g_tcp_server->get_target_q();
                //     //--------follower arm 控制方法1，正规控制-------
                //     // static double q_ref[6]{0};   // TCP更新的目标参考（低频）
                //     // double move[6] = {0.00009, 0.00009, 0.00008, 0.00012, 0.00012, 0.00011};

                //     // // 1) 读取一次TCP目标（可能30Hz更新，但我们500Hz都读也没事）
                //     // auto target = g_tcp_server->get_target_q();

                //     // // 2) 取数据
                //     // if (target.size() >= 6)
                //     // {
                //     //     // 只在数据有效时更新参考
                //     //     for (int i = 0; i < 6; ++i) q_ref[i] = target[i];

                //     //     //if (count() % 500 == 0) mout() << "[DEBUG] m_fd tcp ok" << std::endl;
                //     // }
                //     // else
                //     // {
                //     //     if (count() % 500 == 0) mout() << "[WARN] tcp target_q size < 6, hold last ref" << std::endl;
                //     //     // size不够就保持上一次 q_ref，不要动
                //     // }

                //     // model_a1.setInputPos(q_ref);

                //     // if (model_a1.forwardKinematics())
                //     // {
                //     //     throw std::runtime_error("Forward Kinematics Position Failed!");
                //     // }

                //     // double current_angle[6] = { 0 };

                //     // for (int i = 0; i < 6; i++)
                //     // {
                //     //      current_angle[i] = controller()->motorPool()[i].targetPos();
                //     // }

                //     // for (int i = 0; i < 6; i++)
                //     // {
                //     //     if (current_angle[i] <= q_ref[i] - move[i])
                //     //     {
                //     //         controller()->motorPool()[i].setTargetPos(current_angle[i] + move[i]);
                //     //     }
                //     //     else if (current_angle[i] >= q_ref[i] + move[i])
                //     //     {
                //     //         controller()->motorPool()[i].setTargetPos(current_angle[i] - move[i]);
                //     //     }
                //     //     else
                //     //     {
                //     //         controller()->motorPool()[i].setTargetPos(q_ref[i]);
                //     //     }
                //     // }
                //     //--------follower arm 控制方法12，不正规控制，直接在Aris内部控制-------
                //     eeA1.setV(imp_->v_c);

                //     if (model_a1.inverseKinematicsVel())
                //     {
                //         mout() << "[ERROR] inverseKinematicsVel failed (arm2)" << std::endl;
                //     }
                    
                //     model_a2.setOutputPos(current_pos);
                //     double x_joints[6]{0};
                //     model_a2.getInputPos(x_joints);
                //     for (int i=0; i<6;i++){
                //         controller()->motorPool()[i].setTargetPos(x_joints[i]);
                //     }


                //     //--------舵机控制----------
                //     double gripper_state =target[6];
                //     if(count()%500==0){
                //         std::cout<<"gripper_tcp:"<<gripper_state<<std::endl;
                //     }

                //     if(gripper_state != imp_->gripper_last_state)
                //     {
                //         imp_->gripper_flag = true;
                //         imp_->gripper_last_state = gripper_state;
                //     }
                //     if(imp_->gripper_flag){
                //         if(gripper_state==1){
                //         g_gripper->set_gripper_openclose(10,"100mm", 1, 800);
                //         } else{
                //         g_gripper->set_gripper_openclose(10,"100mm", 0, 800);    
                //         } 
                //         imp_->gripper_flag = false;
                //     }

                // }

            }



		}

        return 30000 - count();
	}
	ForceDrag::ForceDrag(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fd\">"
			"	<GroupParam>"
            "	<Param name=\"model\" default=\"1\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceDrag::~ForceDrag() = default;
	KAANH_DEFINE_BIG_FOUR_CPP(ForceDrag)


    struct Demo::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        double arm1_temp_force[6]{0};

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        double arm2_temp_force[6]{0};

        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };

        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase2_fd[6]{ 5,0,0,0,0,0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        //double K[6]{ 100,100,100,15,15,15 };
        double phase2_B[6]{ 25000,1500,1500,3,3,3 };
        double phase2_M[6]{ 2000,100,100,2,2,2 };


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
        double a_y = -0.0567;
        double b_y = -0.9679;
        double c_y = 0.0010;

        double a_z = 0.9745;
        double b_z = -0.0535;
        double c_z = 0.0063;

        double dy = 0;
        double dz = 0;

        double hole_pos[6]{0};
        double hole_angle[6]{0};



    };
    auto Demo::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
//		gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
//		mout() << "Load Init Force" << std::endl;
//		imp_->init = true;
    }
    auto Demo::executeRT() -> int
    {
        if (!require_dual_arm(*this, "demo"))
        {
            return 0;
        }
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

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_)
        {

            bool isValid = true;
            for (int i = 0; i < 3; i++)
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

                if ((count() - imp_->allign_count) % 30 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->success_count >= count_)
                    {
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



        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {

            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);
            imp_->start_count = count();
            imp_->init = true;

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


            //Phase 1 Contact
            if (!imp_->phase1)
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
                        imp_->phase1 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                               << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
                        break;


                    }

                }
                if (!imp_->phase1)
                {
                    arm2_current_pos[0] -= 0.0000045;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 2 Hold Pos To Get 1s Data
            else if (imp_->phase1 && !imp_->phase2)
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
                if (count() % 1000 == 0)
                {
                    mout() << "force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 8))
                {
                    imp_->phase2 = true;
                    mout() << "Pos Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                           << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                    imp_->complete_count = count();

                }
                else
                {

                    if(arm2_final_force[0] >= (imp_->phase2_fd[0] - 0.01) && arm2_final_force[0] <= (imp_->phase2_fd[0] + 2))
                    {
                        arm2_final_force[0] = imp_->phase2_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase2_fd[i] + arm2_final_force[i] - imp_->phase2_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase2_M[i];
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
            //Phase 3 Get 1s Data to Rough Search
            else if (imp_->phase2 && !imp_->phase3)
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
                        arm1_avg_force[i] = imp_->arm1_temp_force[i] / 30.0;
                        arm2_avg_force[i] = imp_->arm2_temp_force[i] / 30.0;
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
                    imp_->phase3 = true;
                }

            }
            //Phase 4 Back & Move to Hole
            else if (imp_->phase3 && !imp_->phase4)
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


                if(count() <= imp_->back_count + 1000)
                {
                    arm2_current_pos[0] += 0.00001;
                    saMove(arm2_current_pos, model_a2, 1);

                }
                else if (count() == imp_->back_count + 1001)
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
                else if(count() > imp_->back_count + 1001)
                {

                    saJointMove(imp_->hole_angle, 1);
                    if (motorsPositionCheck(current_sa_angle, imp_->hole_angle, 6))
                    {

                        imp_->phase4 = true;
                        imp_->start_count = count();
                        mout()<<"Current Pos: "<< arm2_current_pos[0] <<'\t'<< arm2_current_pos[1] <<'\t' << arm2_current_pos[2]<<std::endl;
                        mout() << "Pos Complete !" << std::endl;
                    }
                }
            }
            //Phase 5 Contact
            else if (imp_->phase4 && !imp_->phase5)
            {
                if(count() == imp_->start_count + 200)
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
                        imp_->phase5 = true;
                        imp_->start_count = 0;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;

                        return 0;

                        break;

                    }

                }
                if (!imp_->phase5)
                {
                    arm2_current_pos[0] -= 0.0000045;
                    saMove(arm2_current_pos, model_a2, 1);
                }
            }


        }


        return 60000 - count();
    }
    Demo::Demo(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"demo\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	<Param name=\"point\" default=\"0\" abbreviation=\"p\"/>"
         "	</GroupParam>"
         "</Command>");
}
    Demo::~Demo() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Demo)




    struct PegOutHole::Imp {

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

        //Switch Model
        int m_;

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double B[6]{ 2500,3500,3500,0,0,0 };
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
    auto PegOutHole::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
        gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
        mout()<<"Load Init Force"<<std::endl;
    }
    auto PegOutHole::executeRT() -> int
    {
        if (!require_dual_arm(*this, "m_po"))
        {
            return 0;
        }
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


        imp_->m_ = int32Param("model");



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



        if(imp_->m_ == 0)
        {
            eeA1.getP(current_pos);
            eeA1.getMpm(current_pm);

             getForceData(actual_force, 0, true);
             gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
             for (size_t i = 0; i < 6; i++)
             {
                comp_force[i] = actual_force[i] + comp_force[i];
             }

             forceTransform(comp_force, transform_force, 0);

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


            current_pos[0] -= d_pos;


            forceUpperLimit(transform_force, limit_area);
            forceDeadZone(transform_force, imp_->deadzone);


            //Impedence Controller
            for (int i = 1; i < 3; i++)
            {
                // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
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
        else if(imp_->m_ == 1)
        {
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


            current_pos[0] += d_pos;


            forceUpperLimit(transform_force, limit_area);
            forceDeadZone(transform_force, imp_->deadzone);


            //Impedence Controller
            for (int i = 1; i < 3; i++)
            {
                // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                acc[i] = (-imp_->f_d[i] + transform_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
            }


            for (int i = 1; i < 3; i++)
            {
                imp_->v_c[i] += acc[i] * dt;
                velDeadZone(imp_->v_c[i], max_vel[i]);
                dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                current_pos[i] = dx[i] + current_pos[i];

            }

            saMove(current_pos, model_a2, 1);
        }
        else
        {
            mout()<<"Wrong Input Model"<<std::endl;
            return 0;
        }




        return 5000 - count();
    }
    PegOutHole::PegOutHole(const std::string& name)
    {
        aris::core::fromXmlString(command(),
         "<Command name=\"m_po\">"
         "	<GroupParam>"
         "	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
         "	</GroupParam>"
         "</Command>");
    }
    PegOutHole::~PegOutHole() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PegOutHole)



    struct Search::Imp {

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

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Desired Pos, Vel, Acc, Foc
        double v_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase4_fd[6]{ 0,0,0,0,0,0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        double phase4_B[6]{ 6000,2500,2500,50,50,50 };
        double phase4_M[6]{ 100,100,100,10,10,10 };

        //Counter
        int search_start_count = 0;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };


        //Search Parameter
        double px[21] = {0,	0.00025, 0.00050, 0.00075, 0.001, 0.00125, 0.00150, 0.00175, 0.002,
        0.00225, 0.0025, 0.00275, 0.003, 0.00325, 0.0035, 0.00375, 0.004, 0.00425, 0.0045, 0.00475, 0.005,};

        double py[21] = {0, -0.0001443, 0.0002887, -0.000433, 0.0005774, -0.0007217, 0.000866, -0.0010104, 0.0011547, -0.001299, 0.0014434, -0.0019485,
        0.0024536, -0.0029587, 0.0034637, -0.0033072, 0.003, -0.0026339, 0.0021794, -0.0015612, 0};

        double force_direction[20]{0};

        double each_count[20] = {194, 300, 432, 570, 711, 854, 996, 1140, 1283, 1427, 1751, 2255, 2759, 3264, 3438, 3206, 2870, 2460, 1925, 841};


        int search_counter = 0;

        //Input Direction
        double y;
        double z;
        double theta;
        double search_pos[2]{0};


    };
    auto Search::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


         GravComp gc;
         gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
         mout() << "Load P & L Vector" << std::endl;
    }
    auto Search::executeRT() -> int
    {
        if (!require_dual_arm(*this, "m_search"))
        {
            return 0;
        }
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
        static double arm2_dead_zone[6]{1.0, 1.0, 1.0, 0, 0, 0};


        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };


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
                if (current_angle[i + 6 * m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i + 6 * m_].setTargetPos(current_angle[i + 6 * m_] + move);
                }
                else if (current_angle[i + 6 * m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i + 6 * m_].setTargetPos(current_angle[i + 6 * m_] - move);
                }
            }
        };

        auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

            model_.setOutputPos(pos_);

            if (model_.inverseKinematics())
            {
                mout() << "Pos: " << pos_[0] << '\t' << pos_[1] << '\t' << pos_[2] << '\t' << pos_[3] << '\t' << pos_[4] << '\t' << pos_[5] << std::endl;
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


        imp_->y = doubleParam("y");
        imp_->z = doubleParam("z");

        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if (!imp_->init)
        {
            double assem_pos[6]{ -0.690, 0.012736, 0.289061, PI / 4, -PI / 2, - PI / 4 };
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
                master()->logFileRawName("Search");


                imp_->theta = std::atan2(imp_->z, imp_->y);

                for(int i = 0; i < 20; i++)
                {
                    imp_->force_direction[i] = std::atan2((imp_->py[i+1] - imp_->py[i]), (imp_->px[i+1] - imp_->px[i])) - imp_->theta;
                }

                mout() << "Init Complete" << std::endl;
                imp_->init = true;
            }


        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            lout()<< "Current Pos: "<< '\t' << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                  << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;


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
            forceDeadZone(arm2_filtered_force, arm2_dead_zone);

            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);


            for (size_t i = 0; i < 3; i++)
            {
                if (abs(arm2_transform_force[i]) > 15.0)
                {
                    mout() << "Emergency Brake" << std::endl;
                    mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                           << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                    return 0;
                 }
             }

            double real_yz_force[2]{0};

            for(int i = 0; i < 2; i++)
            {
                real_yz_force[i] = arm2_transform_force[i+1];
            }

            double expected_force[2]{-5.0,0};

            eeA2.getP(arm2_current_pos);

            if(count() % 50 == 0)
            {
                mout()<<"Current pos: "<<arm2_current_pos[0]<<'\t'<<arm2_current_pos[1]<<'\t'<<arm2_current_pos[2]<<
                '\t'<<"Current force: "<<arm2_transform_force[0]<<'\t'<<arm2_transform_force[1]<<'\t'<<arm2_transform_force[2]<<std::endl;
            }


            if(imp_->search_counter < 20)
            {
                //Only yz movment
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

                    //Impedence Controller
                    for (int i = 0; i < 2; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-expected_force[i] + trans_yz_force[i] - imp_->phase4_B[i+1] * (imp_->v_c[i+1] - imp_->v_d[i+1])) / imp_->phase4_M[i+1];
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

                    acc_x = (-imp_->phase4_fd[0] + arm2_transform_force[0] - imp_->phase4_B[0] * (imp_->v_c[0] - imp_->v_d[0])) / imp_->phase4_M[0];
                    imp_->v_c[0] += acc_x * dt;
                    velDeadZone(imp_->v_c[0], max_vel[0]);
                    dx_x = imp_->v_c[0] * dt + acc_x * dt * dt;

                    arm2_current_pos[0] += dx_x;

                    arm2_current_pos[1] += trans_dx[0];
                    arm2_current_pos[2] += trans_dx[1];

                    imp_->search_start_count++;


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


        return 60000 - count();
    }
    Search::Search(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"m_search\">"
            "	<GroupParam>"
            "	<Param name=\"y\" default=\"0\" abbreviation=\"y\"/>"
            "	<Param name=\"z\" default=\"0\" abbreviation=\"z\"/>"
            "	</GroupParam>"
            "</Command>");
    }
    Search::~Search() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Search)


// Impedence Controller
    struct Arm2PegInHole::Imp {

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


        //Desired Force of Each Phase

        double phase3_fd[6]{ 5.0,0,0,0,0,0 };
        double phase4_fd[6]{ 3.5,0,0,0,0,0 };
        double phase5_fd[6]{ 3.5,0,0,0,0,0 };
        double phase6_fd[6]{ 7.5,0,0,0,0,0 };

        //Current Vel && Desired Vel
        double v_c[6]{ 0 };
        double v_d[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 3000,2000,2000,0,0,0 };
        double phase3_M[6]{ 200,100,100,0,0,0 };

        //4.5 2.5
        double phase4_B[6]{ 3000,3500,3500,4.5,3.5,3.5 };
        double phase4_M[6]{ 150,100,100,2.5,1.0,1.0 };

        double phase5_B[6]{ 4000,3500,3500,0,0,0 };
        double phase5_M[6]{ 100,100,100,0,0,0 };

        double phase6_B[6]{ 3500,3500,3500,15,80,80 };
        double phase6_M[6]{ 100,100,100,10,5,5 };

        //Force Counter
        int allign_count = 0;
        int success_count = 0;

        //Counter for force comp
        int start_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        //Data Collect Counter
        int complete_count = 0;
        int back_count = 0;
        int accumulation_count = 0;

        double current_pos_checkek[6] = {0};

        //Parameters For Compensate
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;

        //Switch Angle
        double z_;
        double y_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Dead Zone 8
        double p3_deadzone[6]{0.1,3,3,0,0,0};
        double p4_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p5_deadzone[6]{0.1,3,3,0,0,0};
        double p6_deadzone[6]{0.1,5,5,0.3,0.3,0.3};

        //2-Points Contact Pos
        double tpcontact_pos[6]{0};



    };
    auto Arm2PegInHole::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto Arm2PegInHole::executeRT() -> int
    {
        if (!require_dual_arm(*this, "a2ph"))
        {
            return 0;
        }
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

        double mz_comp = 0;


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
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
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
            //Phase 2 Contact
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
                    arm2_current_pos[0] -= 0.0000035;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Maintain 2 Points Contact
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

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
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
                    forceDeadZone(arm2_final_force, imp_->p3_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase3_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase3_fd[0] + 5.0))
                    {
                        arm2_final_force[0] = imp_->phase3_fd[0];
                    }




                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
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
            //Phase 4 allign
            else if (imp_->phase3 && !imp_->phase4)
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

                        mout() << "Brake force Phase3: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
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

                if (depth >= 0.005 && forceCheck(arm2_final_force, desired_force, 5, 3))
                {
                    imp_->phase4 = true;
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
                    forceDeadZone(arm2_final_force, imp_->p4_deadzone);

//                    if (count() % 100 == 0)
//                    {
//                        mout() << "compf: " << arm2_final_force[0] << '\t' << arm2_final_force[1] << '\t' << arm2_final_force[2] << '\t'
//                            << arm2_final_force[3] << '\t' << arm2_final_force[4] << '\t' << arm2_final_force[5] << std::endl;
//                    }


                    if(arm2_final_force[0] >= (imp_->phase4_fd[0] - 2) && arm2_final_force[0] <= (imp_->phase4_fd[0] + 5.5))
                    {
                        arm2_final_force[0] = imp_->phase4_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + arm2_final_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
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
                        ome[i] = (-imp_->phase4_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase4_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase4_M[i + 3];
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
            //Phase 5 Maintain Contact
            else if(imp_->phase4 && !imp_->phase5)
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

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase5 = true;
                    mout() << "Pos 5 Complete" << std::endl;
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
                    forceDeadZone(arm2_final_force, imp_->p5_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase5_fd[0] - 2.0) && arm2_final_force[0] <= (imp_->phase5_fd[0] + 10.0))
                    {
                        arm2_final_force[0] = imp_->phase5_fd[0];
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
            //Phase 6 Allign
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
                    imp_->phase6 = true;
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
                    forceDeadZone(arm2_final_force, imp_->p6_deadzone);


                    if(arm2_final_force[0] >= (imp_->phase6_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase6_fd[0] + 5.5))
                    {
                        arm2_final_force[0] = imp_->phase6_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase6_fd[i] + arm2_final_force[i] - imp_->phase6_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase6_M[i];
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
                        ome[i] = (-imp_->phase6_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase6_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase6_M[i + 3];
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


        return 150000 - count();
    }
    Arm2PegInHole::Arm2PegInHole(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"a2ph\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    Arm2PegInHole::~Arm2PegInHole() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(Arm2PegInHole)


    struct PlateAllignData::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;


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


        //Desired Force of Each Phase

        double phase3_fd[6]{ 3.5,0,0,0,0,0 };
        double phase4_fd[6]{ 3.5,0,0,0,0,0 };
        double phase5_fd[6]{ 3.5,0,0,0,0,0 };
        double phase6_fd[6]{ 3.5,0,0,0,0,0 };

        //Current Vel && Desired Vel
        double v_c[6]{ 0 };
        double v_d[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 4000,3300,3300,0,0,0 };
        double phase3_M[6]{ 100,100,100,0,0,0 };

        //4.5 2.5
        double phase4_B[6]{ 3000,3500,3500,4.5,3.5,3.5 };
        double phase4_M[6]{ 150,100,100,2.5,1.0,1.0 };

        double phase5_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase5_M[6]{100, 100, 100, 25, 25, 25};

        double phase6_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase6_M[6]{100, 100, 100, 25, 25, 25};

        //Force Counter
        int allign_count = 0;
        int success_count = 0;

        //Counter for force comp
        int start_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        //Data Collect Counter
        int complete_count = 0;
        int back_count = 0;
        int accumulation_count = 0;

        double current_pos_checkek[6] = {0};

        //Parameters For Compensate
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;

        //Switch Angle
        double z_;
        double y_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Dead Zone 8
        double p3_deadzone[6]{0.1,3,3,0,0,0};
        double p4_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p5_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p6_deadzone[6]{0,0,0,0.05,0.015,0.015};

        //2-Points Contact Pos
        double tpcontact_pos[6]{0};



    };
    auto PlateAllignData::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto PlateAllignData::executeRT() -> int
    {
        if (!require_dual_arm(*this, "m_pad"))
        {
            return 0;
        }
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

        double mz_comp = 0;


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
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
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
            double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0  };
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
                double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0 };
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
            //Phase 2 Contact
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
                    arm2_current_pos[0] -= 0.0000035;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Maintain 2 Points Contact
            else if (imp_->phase2 && !imp_->phase3)
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

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
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
                    forceDeadZone(arm2_final_force, imp_->p3_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase3_fd[0] - 2.0) && arm2_final_force[0] <= (imp_->phase3_fd[0] + 10.0))
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
            //Phase 4 Get Data
            else if (imp_->phase3 && !imp_->phase4)
            {
                if(count() <= imp_->complete_count + 1550)
                {
                    if(count() % 50 == 0 && imp_->accumulation_count < 30)
                    {
                        mout()<<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                               <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;


                        for(int i = 0; i<6; i++)
                        {
                            imp_->arm1_temp_force[i] += arm1_transform_force[i];
                            imp_->arm2_temp_force[i] += arm2_transform_force[i];

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
                        arm1_avg_force[i] = imp_->arm1_temp_force[i] / 30.0;
                        arm2_avg_force[i] = imp_->arm2_temp_force[i] / 30.0;
                    }


                    mout()<<"Data Acquired!"  << '\t' <<"Current Z Angle: "<< imp_->z_ << '\t' <<"Current Y Angle: "<< imp_->y_ <<std::endl;

                    mout()<<"A2_Force"<<"\t"<<arm2_avg_force[0]<<"\t"<<arm2_avg_force[1]<<"\t"<<arm2_avg_force[2]<<"\t"
                           <<arm2_avg_force[3]<<"\t"<<arm2_avg_force[4]<<"\t"<<arm2_avg_force[5]<<std::endl;

                    imp_->back_count = count();
                    imp_->accumulation_count = 0;
                    imp_->phase4 = true;
                }

            }
            //Back
            else if(imp_->phase4 && !imp_->phase5)
            {
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


                if(count() <= imp_->back_count + 4000)
                {
                    arm2_current_pos[0] += 0.00001;
                    saMove(arm2_current_pos, model_a2, 1);
                }
                else
                {
                    mout()<<"Test Complete! "<< '\t' <<"Current Z Angle: "<< imp_->z_ << '\t' <<"Current Y Angle: "<< imp_->y_ <<std::endl;
                    imp_->phase5 = true;
                    return 0;
                }
            }


        }


        return 150000 - count();
    }
    PlateAllignData::PlateAllignData(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_pad\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    PlateAllignData::~PlateAllignData() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PlateAllignData)

    struct PlateAllignTest::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;

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


        //Desired Force of Each Phase

        double phase3_fd[6]{ 3.5,0,0,0,0,0 };
        double phase4_fd[6]{ 7.5,0,0,0,0,0 };
        double phase5_fd[6]{ 3.5,0,0,0,0,0 };
        double phase6_fd[6]{ 3.5,0,0,0,0,0 };

        //Current Vel && Desired Vel
        double v_c[6]{ 0 };
        double v_d[6]{ 0 };

        //Impedence Parameter
        double phase3_B[6]{ 4000,3300,3300,0,0,0 };
        double phase3_M[6]{ 100,100,100,0,0,0 };

        //4.5 2.5
        double phase4_B[6]{ 3500,2500,2500,15,80,80 };
        double phase4_M[6]{ 100,100,100,10,5,5 };

        double phase5_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase5_M[6]{100, 100, 100, 25, 25, 25};

        double phase6_B[6]{3000, 3300, 3300, 45, 45, 45};
        double phase6_M[6]{100, 100, 100, 25, 25, 25};

        //Force Counter
        int allign_count = 0;
        int success_count = 0;

        //Counter for force comp
        int start_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        //Data Collect Counter
        int complete_count = 0;
        int back_count = 0;
        int accumulation_count = 0;

        double current_pos_checkek[6] = {0};

        //Parameters For Compensate
        double a_y = -0.0592;
        double b_y = -0.9943;
        double c_y = 0.0132;

        //Switch Angle
        double z_;
        double y_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

        //Dead Zone 8
        double p3_deadzone[6]{0.1,3,3,0,0,0};
        double p4_deadzone[6]{0.1,0.1,0.1,0.3,0.3,0.3};
        double p5_deadzone[6]{0,0,0,0.05,0.015,0.015};
        double p6_deadzone[6]{0,0,0,0.05,0.015,0.015};

        //2-Points Contact Pos
        double tpcontact_pos[6]{0};



    };
    auto PlateAllignTest::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
    }
    auto PlateAllignTest::executeRT() -> int
    {
        if (!require_dual_arm(*this, "m_pat"))
        {
            return 0;
        }
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

        double mz_comp = 0;


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
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
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
            double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0  };
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
                double assem_pos[6]{ -0.710, 0.012466, 0.291196, 0.0142609, -1.56474, 0.0 };
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
            //Phase 2 Contact Check
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
                    arm2_current_pos[0] -= 0.0000035;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Maintain Contact
            else if (imp_->phase2 && !imp_->phase3)
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

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos 3 Complete" << std::endl;
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
                    forceDeadZone(arm2_final_force, imp_->p3_deadzone);

                    if(arm2_final_force[0] >= (imp_->phase3_fd[0] - 2.0) && arm2_final_force[0] <= (imp_->phase3_fd[0] + 10.0))
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
            //Phase 4 Allign
            else if (imp_->phase3 && !imp_->phase4)
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

                if (posCheck(arm2_current_pos, 10))
                {
                    imp_->phase4 = true;
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
                    forceDeadZone(arm2_final_force, imp_->p4_deadzone);


                    if(arm2_final_force[0] >= (imp_->phase4_fd[0] - 2.5) && arm2_final_force[0] <= (imp_->phase4_fd[0] + 5.5))
                    {
                        arm2_final_force[0] = imp_->phase4_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + arm2_final_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


                    for (int i = 0; i < 1; i++)
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
                        ome[i] = (-imp_->phase4_fd[i + 3] + arm2_final_force[i + 3] - imp_->phase4_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase4_M[i + 3];
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
            //Back
            else if(imp_->phase4 && !imp_->phase5)
            {

            }


        }


        return 150000 - count();
    }
    PlateAllignTest::PlateAllignTest(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_pat\">"
         "	<GroupParam>"
         "	<Param name=\"z_degree\" default=\"0\" abbreviation=\"z\"/>"
         "	<Param name=\"y_degree\" default=\"0\" abbreviation=\"y\"/>"
         "	</GroupParam>"
         "</Command>");
}
    PlateAllignTest::~PlateAllignTest() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PlateAllignTest)



// PressDown 类的实现
	struct PressDown::Imp
{
	bool inited = false;

	double start_pos[6]{0};
	double target_pos[6]{0};
	double moved_down = 0.0;

	double max_down = 0.02;      // 2 cm
	double v_down  = 0.005;       // 1 cm/s
	double dt      = 0.002;      // 2 ms
	double t_limit = 10.0;       // 10 s timeout

	double k_sim = 600.0;        // N/m
	double fz_threshold = 10.0;  // 10 N

	double simFz() const { return k_sim * moved_down; }
};

auto PressDown::prepareNrt() -> void
{
	// [MOD] 仅保留与其他命令一致的 motorOptions 设置，不做任何额外东西
	for (auto &m : motorOptions())
		m = aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
}

auto PressDown::executeRT() -> int
{
	// ===================== [MOD] 首周期重置（替代prepareNrt里的重置） =====================
	// 这样每次运行本命令，都从头开始，不会继承上一次imp_的状态
	if (count() == 1)
	{
		imp_->inited = false;
		imp_->moved_down = 0.0;
	}

	// dual transform modelbase into multimodel
	auto &dualArm = dynamic_cast<aris::dynamic::MultiModel &>(modelBase()[0]);
	auto &arm1 = dualArm.subModels().at(0);
	auto &model_a1 = dynamic_cast<aris::dynamic::Model &>(arm1);
	auto &eeA1 = dynamic_cast<aris::dynamic::GeneralMotion &>(model_a1.generalMotionPool().at(0));
	(void)eeA1;
	// =====================================================================================

	// [MOD] 固定选择 m=0（Arm1）
	auto &model = model_a1;
	auto &ee    = eeA1;
	const int motor_start_idx = 0; // Arm1 motors: 0~5

	// [MOD] 运行时间
	const double t = count() * imp_->dt;

	// [MOD] 停止条件3：超时10秒
	if (t >= imp_->t_limit)
	{
		mout() << "PressDown(sim): STOP timeout t=" << t << "s" << std::endl;
		return 0;
	}

	// [MOD] 第一次进入：记录起始末端位姿
	if (!imp_->inited)
	{
		ee.getP(imp_->start_pos);
		std::copy(imp_->start_pos, imp_->start_pos + 6, imp_->target_pos);
		imp_->moved_down = 0.0;
		imp_->inited = true;
	}

	// [MOD] 停止条件2：下压位移超过2cm
	if (imp_->moved_down >= imp_->max_down)
	{
		mout() << "PressDown(sim): STOP down=" << imp_->moved_down << "m" << std::endl;
		return 0;
	}

	// [MOD] 仿真力（预设，不读传感器）
	const double fz_sim = imp_->simFz();

	// [MOD] 停止条件1：仿真Z轴力超过10N
	if (fz_sim > imp_->fz_threshold)
	{
		mout() << "PressDown(sim): STOP Fz_sim=" << fz_sim << "N" << std::endl;
		return 0;
	}

	// [MOD] 本周期位移步进：v * dt
	const double step = imp_->v_down * imp_->dt;
	const double remain = imp_->max_down - imp_->moved_down;
	const double real_step = (step < remain) ? step : remain;
	imp_->moved_down += real_step;

	// [MOD] 目标位姿：仅Z向下
	std::copy(imp_->start_pos, imp_->start_pos + 6, imp_->target_pos);
	imp_->target_pos[2] = imp_->start_pos[2] - imp_->moved_down;

	// [MOD] IK + 下发关节目标
	model.setOutputPos(imp_->target_pos);
	if (model.inverseKinematics())
	{
		mout() << "PressDown(sim): IK failed, STOP" << std::endl;
		return 0;
	}

	double q[6]{0};
	model.getInputPos(q);
	const int n_axis = axis_count(*this);
	for (int i = 0; i < n_axis; ++i)
		controller()->motorPool()[motor_start_idx + i].setTargetPos(q[i]);

	// 低频打印（每1秒一次）
	if (count() % 500 == 0)
	{
		mout() << "PressDown(sim): t=" << t << "s, down=" << imp_->moved_down
		       << "m, Fz_sim=" << fz_sim << "N" << std::endl;
	}

	return 1;
}

	PressDown::PressDown(const std::string &name) : imp_(new Imp)
	{
		// 注册命令，与xml中的命令名称对应
		aris::core::fromXmlString(command(),
			"<Command name=\"press_down\">"
			"	<GroupParam>"
			"		<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"		<Param name=\"tool\" default=\"0\" abbreviation=\"t\"/>"
			"	</GroupParam>"
			"</Command>");
	}

	PressDown::~PressDown() = default;

    struct JointTrajCli::Imp {
        int traj_id{1};
        int model_id{0};
    };

    auto JointTrajCli::prepareNrt() -> void {
        for (auto &m : motorOptions()) {
            m = aris::plan::Plan::CHECK_NONE |
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }
    }

    auto JointTrajCli::executeRT() -> int {
        const double kDeg = PI / 180.0;
        using JointPoint = std::array<double, 6>;
        static const std::unordered_map<int, std::vector<JointPoint>> traj_db{
            {1, {
                {{-26 * kDeg, -31 * kDeg, 139 * kDeg, -77 * kDeg, -66 * kDeg, -13 * kDeg}},
                {{-33 * kDeg, -16 * kDeg, 135 * kDeg, -83 * kDeg, -67 * kDeg, -16 * kDeg}},
                {{-24 * kDeg,  1 * kDeg, 140 * kDeg, -108 * kDeg, -74 * kDeg, -10 * kDeg}},
                {{-22 * kDeg, 17 * kDeg, 135 * kDeg, -126 * kDeg, -74 * kDeg, -8 * kDeg}}
            }},
            {2, {
                {{0, 0, 0, 0, 0, 0}},
                {{-15 * kDeg, 25 * kDeg, -20 * kDeg, 10 * kDeg, -15 * kDeg, 5 * kDeg}},
                {{-5 * kDeg, 10 * kDeg, -35 * kDeg, -10 * kDeg, 10 * kDeg, -5 * kDeg}},
                {{0, 0, 0, 0, 0, 0}}
            }},
            {3, {
                {{0, 0, 0, 0, 0, 0}},
                {{8 * kDeg, 8 * kDeg, 8 * kDeg, 0, 0, 0}},
                {{-8 * kDeg, -8 * kDeg, -8 * kDeg, 0, 0, 0}},
                {{8 * kDeg, -8 * kDeg, 8 * kDeg, 0, 0, 0}},
                {{0, 0, 0, 0, 0, 0}}
            }}
        };

        if (count() == 1) {
            imp_->traj_id = int32Param("traj");
            imp_->model_id = int32Param("model");
        }

        if (imp_->model_id < 0 || imp_->model_id > 1) {
            mout() << "traj_run: model must be 0 or 1" << std::endl;
            return 0;
        }
        if (static_cast<std::size_t>(imp_->model_id) >= submodel_count(*this)) {
            mout() << "traj_run: model " << imp_->model_id << " not in this robot" << std::endl;
            return 0;
        }

        const int motor_start = imp_->model_id * 6;
        if (controller()->motorPool().size() < static_cast<aris::Size>(motor_start + 6)) {
            mout() << "traj_run: motor pool size not enough" << std::endl;
            return 0;
        }

        const auto it = traj_db.find(imp_->traj_id);
        if (it == traj_db.end()) {
            mout() << "traj_run: unsupported traj id, use 1/2/3" << std::endl;
            return 0;
        }
        const auto &pts = it->second;
        const int pts_size = static_cast<int>(pts.size());
        if (pts_size < 2) {
            mout() << "traj_run: trajectory points must be >= 2" << std::endl;
            return 0;
        }

        // Keep fixed internal timing like m_init: no external speed parameter.
        constexpr int kCyclesPerSeg = 4000;
        const int total_cycles = (pts_size - 1) * kCyclesPerSeg;
        if (count() > total_cycles) return 0;

        const int c = count() - 1;
        const int seg = c / kCyclesPerSeg;
        if (seg >= pts_size - 1) return 0;

        const double u = static_cast<double>((c % kCyclesPerSeg) + 1) /
                         static_cast<double>(kCyclesPerSeg);
        // Smoothstep interpolation to ensure zero velocity at endpoints.
        const double alpha = u * u * (3.0 - 2.0 * u);
        for (int i = 0; i < 6; ++i) {
            const double q = pts[seg][i] + (pts[seg + 1][i] - pts[seg][i]) * alpha;
            controller()->motorPool()[motor_start + i].setTargetPos(q);
        }

        if (count() % 500 == 0) {
            mout() << "traj_run: traj=" << imp_->traj_id
                   << ", model=" << imp_->model_id
                   << ", seg=" << seg + 1 << "/" << (pts_size - 1) << std::endl;
        }
        return total_cycles - count();
    }

    JointTrajCli::JointTrajCli(const std::string &name) : imp_(new Imp) {
        aris::core::fromXmlString(command(),
            "<Command name=\"traj_run\">"
            "  <GroupParam>"
            "    <Param name=\"traj\" abbreviation=\"t\" default=\"1\"/>"
            "    <Param name=\"model\" abbreviation=\"m\" default=\"0\"/>"
            "  </GroupParam>"
            "</Command>");
    }

    JointTrajCli::~JointTrajCli() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(JointTrajCli)

    struct PoseTrajCli::Imp {
        int traj_id{1};
        int model_id{0};
        double start_pe[6]{0};
    };

    auto PoseTrajCli::prepareNrt() -> void {
        for (auto &m : motorOptions()) {
            m = aris::plan::Plan::CHECK_NONE |
                aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        }
    }

    auto PoseTrajCli::executeRT() -> int {
        const double kDeg = PI / 180.0;
        using PosePoint = std::array<double, 6>; // [x,y,z,rx,ry,rz], m + rad
        static const std::unordered_map<int, std::vector<PosePoint>> pose_traj_db{
            {1, {
                {{604.0 / 1000.0, -121.0 / 1000.0, 248.0 / 1000.0, 359 * kDeg, -49 * kDeg, 181 * kDeg}}
            }},
            {2, {
                {{634.0 / 1000.0, -32.0 / 1000.0, 216.0 / 1000.0, 349 * kDeg, -55 * kDeg, 186 * kDeg}}
            }},
            {3, {
                {{559.0 / 1000.0, 14.0 / 1000.0, 230.0 / 1000.0, 359 * kDeg, -47 * kDeg, 181 * kDeg}}
            }},
            {4, {
                {{587.0 / 1000.0, 88.0 / 1000.0, 252.0 / 1000.0, 348 * kDeg, -48 * kDeg, 187 * kDeg}}
            }},
            {5, {
                {{527.0 / 1000.0, 132.0 / 1000.0, 231.0 / 1000.0, 347 * kDeg, -53 * kDeg, 188 * kDeg}}
            }},
            {6, {
                {{577.0 / 1000.0, 130.0 / 1000.0, 241.0 / 1000.0, 347 * kDeg, -51 * kDeg, 198 * kDeg}}
            }},
            {7, {
                {{566.0 / 1000.0, 217.0 / 1000.0, 222.0 / 1000.0, 348 * kDeg, -47 * kDeg, 196 * kDeg}}
            }},
            {8, {
                {{688.0 / 1000.0, 206.0 / 1000.0, 241.0 / 1000.0, 355 * kDeg, -47 * kDeg, 196 * kDeg}}
            }},
            {9, {
                {{760.0 / 1000.0, 161.0 / 1000.0, 252.0 / 1000.0, 354 * kDeg, -40 * kDeg, 194 * kDeg}}
            }},
            {10, {
                {{702.0 / 1000.0, 111.0 / 1000.0, 252.0 / 1000.0, 353 * kDeg, -40 * kDeg, 188 * kDeg}}
            }},
            {11, {
                {{762.0 / 1000.0, 65.0 / 1000.0, 252.0 / 1000.0, 360 * kDeg, -37 * kDeg, 188 * kDeg}}
            }},
            {12, {
                {{677.0 / 1000.0, 22.0 / 1000.0, 252.0 / 1000.0, 351 * kDeg, -45 * kDeg, 189 * kDeg}}
            }},
            {13, {
                {{752.0 / 1000.0, -9.0 / 1000.0, 237.0 / 1000.0, 344 * kDeg, -45 * kDeg, 189 * kDeg}}
            }},
            {14, {
                {{696.0 / 1000.0, -49.0 / 1000.0, 225.0 / 1000.0, 338 * kDeg, -48 * kDeg, 191 * kDeg}}
            }},
            {15, {
                {{822.0 / 1000.0, -64.0 / 1000.0, 234.0 / 1000.0, 335 * kDeg, -47 * kDeg, 187 * kDeg}}
            }},
            {16, {
                {{819.0 / 1000.0, -9.0 / 1000.0, 271.0 / 1000.0, 352 * kDeg, -46 * kDeg, 181 * kDeg}}
            }},
            {17, {
                {{842.0 / 1000.0, 56.0 / 1000.0, 289.0 / 1000.0, 356 * kDeg, -33 * kDeg, 180 * kDeg}}
            }},
            {18, {
                {{859.0 / 1000.0, 105.0 / 1000.0, 302.0 / 1000.0, 356 * kDeg, -27 * kDeg, 180 * kDeg}}
            }},
            {19, {
                {{846.0 / 1000.0, 138.0 / 1000.0, 298.0 / 1000.0, 359 * kDeg, -29 * kDeg, 194 * kDeg}}
            }},
            {20, {
                {{850.0 / 1000.0, 208.0 / 1000.0, 287.0 / 1000.0, 358 * kDeg, -28 * kDeg, 197 * kDeg}}
            }},
            {21, {
                {{876.0 / 1000.0, 208.0 / 1000.0, 256.0 / 1000.0, 359 * kDeg, -46 * kDeg, 196 * kDeg}}
            }},
            {22, {
                {{893.0 / 1000.0, 147.0 / 1000.0, 256.0 / 1000.0, 363 * kDeg, -46 * kDeg, 188 * kDeg}}
            }},
            {23, {
                {{893.0 / 1000.0, 64.0 / 1000.0, 277.0 / 1000.0, 363 * kDeg, -41 * kDeg, 188 * kDeg}}
            }},
            {24, {
                {{918.0 / 1000.0, 9.0 / 1000.0, 285.0 / 1000.0, 350 * kDeg, -36 * kDeg, 187 * kDeg}}
            }},
            {25, {
                {{936.0 / 1000.0, -16.0 / 1000.0, 271.0 / 1000.0, 344 * kDeg, -40 * kDeg, 175 * kDeg}}
            }}
        };

        if (count() == 1) {
            imp_->traj_id = int32Param("traj");
            imp_->model_id = int32Param("model");
        }

        if (imp_->model_id < 0 || imp_->model_id > 1) {
            mout() << "ptr: model must be 0 or 1" << std::endl;
            return 0;
        }
        if (static_cast<std::size_t>(imp_->model_id) >= submodel_count(*this)) {
            mout() << "ptr: model " << imp_->model_id << " not in this robot" << std::endl;
            return 0;
        }

        const auto it = pose_traj_db.find(imp_->traj_id);
        if (it == pose_traj_db.end()) {
            mout() << "ptr: unsupported traj id, use 1..25" << std::endl;
            return 0;
        }
        const auto &pts = it->second;
        const int pts_size = static_cast<int>(pts.size());
        if (pts_size < 1) {
            mout() << "ptr: trajectory points must be >= 1" << std::endl;
            return 0;
        }

        auto &multi_model = dynamic_cast<aris::dynamic::MultiModel &>(modelBase()[0]);
        auto &sub_model = dynamic_cast<aris::dynamic::Model &>(multi_model.subModels().at(imp_->model_id));
        auto &ee = dynamic_cast<aris::dynamic::GeneralMotion &>(sub_model.generalMotionPool().at(0));

        if (count() == 1) {
            ee.getP(imp_->start_pe);
        }

        const int motor_start = imp_->model_id * 6;
        if (controller()->motorPool().size() < static_cast<aris::Size>(motor_start + 6)) {
            mout() << "ptr: motor pool size not enough" << std::endl;
            return 0;
        }

        constexpr int kCyclesPerSeg = 4000;
        const int total_cycles = pts_size * kCyclesPerSeg;
        if (count() > total_cycles) return 0;

        const int c = count() - 1;
        const int seg = c / kCyclesPerSeg;
        if (seg >= pts_size) return 0;

        const double u = static_cast<double>((c % kCyclesPerSeg) + 1) /
                         static_cast<double>(kCyclesPerSeg);
        // Smoothstep interpolation to ensure zero velocity at endpoints.
        const double alpha = u * u * (3.0 - 2.0 * u);

        const double *start = (seg == 0) ? imp_->start_pe : pts[seg - 1].data();
        const double *end = pts[seg].data();

        double target_pe[6]{0};
        for (int i = 0; i < 6; ++i) {
            target_pe[i] = start[i] + (end[i] - start[i]) * alpha;
        }

        sub_model.setOutputPos(target_pe);
        if (sub_model.inverseKinematics()) {
                mout() << "ptr: IK failed" << std::endl;
            return 0;
        }

        double q[6]{0};
        sub_model.getInputPos(q);
        for (int i = 0; i < 6; ++i) {
            controller()->motorPool()[motor_start + i].setTargetPos(q[i]);
        }

        if (count() % 500 == 0) {
            mout() << "ptr: traj=" << imp_->traj_id
                   << ", model=" << imp_->model_id
                   << ", seg=" << seg + 1 << "/" << pts_size << std::endl;
        }
        return total_cycles - count();
    }

    PoseTrajCli::PoseTrajCli(const std::string &name) : imp_(new Imp) {
        aris::core::fromXmlString(command(),
            "<Command name=\"ptr\">"
            "  <GroupParam>"
            "    <Param name=\"traj\" abbreviation=\"t\" default=\"1\"/>"
            "    <Param name=\"model\" abbreviation=\"m\" default=\"0\"/>"
            "  </GroupParam>"
            "</Command>");
    }

    PoseTrajCli::~PoseTrajCli() = default;
    KAANH_DEFINE_BIG_FOUR_CPP(PoseTrajCli)










    ARIS_REGISTRATION {
        aris::core::class_<ModelInit>("ModelInit")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelForward>("ModelForward")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelGet>("ModelGet")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelTest>("ModelTest")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelSetPos>("ModelSetPos")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ModelComP>("ModelComP")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ForceAlign>("ForceAlign")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ForceKeep>("ForceKeep")
            .inherit<aris::plan::Plan>();
        aris::core::class_<ForceDrag>("ForceDrag")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PegOutHole>("PegOutHole")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Demo>("Demo")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Search>("Search")
            .inherit<aris::plan::Plan>();
        aris::core::class_<Arm2PegInHole>("Arm2PegInHole")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PlateAllignData>("PlateAllignData")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PlateAllignTest>("PlateAllignTest")
            .inherit<aris::plan::Plan>();

        aris::core::class_<PressDown>("PressDown")
			.inherit<aris::plan::Plan>();
        aris::core::class_<JointTrajCli>("JointTrajCli")
            .inherit<aris::plan::Plan>();
        aris::core::class_<PoseTrajCli>("PoseTrajCli")
            .inherit<aris::plan::Plan>();

    }


}
