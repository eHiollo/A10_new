#include "test.hpp"
#include "gravcomp.hpp"
#include <array>

using namespace std;

namespace testSpace
{

	struct ZeroG::Imp
	{
		double x[10000]{0};
		double y[10000]{0};
		double z[10000]{0};

		double vx[10000]{0};
		double vy[10000]{0};
		double vz[10000]{0};

		double ax[10000]{0};
		double ay[10000]{0};
		double az[10000]{0};

		double u[10000]{0};
		double v[10000]{0};
		double w[10000]{0};

		double vu[10000]{0};
		double vv[10000]{0};
		double vw[10000]{0};

		double au[10000]{0};
		double av[10000]{0};
		double aw[10000]{0};

		int mycount1 = 0;
		int mycount2 = 0;

		double finalPos[6]{0};

		bool init = false;
		bool contact_check = false;

		double arm1_init_force[6]{0};
		double arm1_start_force[6]{0};
		double arm1_p_vector[6]{0};
		double arm1_l_vector[6]{0};

		double arm2_init_force[6]{0};
		double arm2_start_force[6]{0};
		double arm2_p_vector[6]{0};
		double arm2_l_vector[6]{0};

		int m_;

		std::array<double, 10> arm1_force_buffer[6] = {};
		int arm1_buffer_index[6]{0};

		std::array<double, 10> arm2_force_buffer[6] = {};
		int arm2_buffer_index[6]{0};
	};
	auto ZeroG::prepareNrt() -> void
	{

		for (auto &m : motorOptions())
			m =
				aris::plan::Plan::CHECK_NONE |
				aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ZeroG::executeRT() -> int
	{
		// dual transform modelbase into multimodel
		auto &dualArm = dynamic_cast<aris::dynamic::MultiModel &>(modelBase()[0]);
		// at(0) -> Arm1 -> white
		auto &arm1 = dualArm.subModels().at(0);
		// at(1) -> Arm2 -> blue
		auto &arm2 = dualArm.subModels().at(1);

		// transform to model
		auto &model_a1 = dynamic_cast<aris::dynamic::Model &>(arm1);
		auto &model_a2 = dynamic_cast<aris::dynamic::Model &>(arm2);

		// End Effector
		auto &eeA1 = dynamic_cast<aris::dynamic::GeneralMotion &>(model_a1.generalMotionPool().at(0));
		auto &eeA2 = dynamic_cast<aris::dynamic::GeneralMotion &>(model_a2.generalMotionPool().at(0));

		// xmax = 1.332887; xmin = -0.721634
		// ymax = 0.926673; ymin = -1.12283
		// zmax = 1.37994; zmin = -1.10199
		// init  0.530866  0.124673  0.34254

		// F = 1; max   1.03037  0.624173  0.84204
		// F = -1; min  0.361036  -0.045157  0.17271

		GravComp gc;

		static double mass = 200;
		static double Ix = 2.1815;
		static double Iy = 4.3630;
		static double Iz = 5.6719;
		/* static double mass = 50;
		static double Ix = 0.4193;
		static double Iy = 0.4193;
		static double Iz = 0.4193; */
		static double saveXYZ[6]{0};
		static double mmXYZ[6]{0};
		int savetype = 0;
		// int whichtype[3]{0};
		int isvwSave = 0;

		double currentForce[6]{0};
		double currentPos[6]{0};
		double targetPos[6]{0};
		double targetAngle[6]{0};
		double currentv[3]{0};
		double currentw[3]{0};

		double domega[3]{0};
		double drm[9]{0};
		double currentrm[9]{0};
		double targetrm[9]{0};

		double arm1_comp_force[6]{0};
		double arm1_current_pm[16]{0};
		double arm1_current_pos[6]{0};
		double arm1_actual_force[6]{0};
		double arm1_filtered_force[6]{0};
		double arm1_transform_force[6]{0};

		auto obtainX = [&](double Force)
		{
			imp_->ax[count()] = (Force / mass);
			imp_->vx[count()] = (imp_->vx[count() - 1] + imp_->ax[count()] * 0.001);
			imp_->x[count()] = (imp_->x[count() - 1] + imp_->vx[count()] * 0.001);
		};
		auto obtainY = [&](double Force)
		{
			imp_->ay[count()] = (Force / mass);
			imp_->vy[count()] = (imp_->vy[count() - 1] + imp_->ay[count()] * 0.001);
			imp_->y[count()] = (imp_->y[count() - 1] + imp_->vy[count()] * 0.001);
		};
		auto obtainZ = [&](double Force)
		{
			imp_->az[count()] = (Force / mass);
			imp_->vz[count()] = (imp_->vz[count() - 1] + imp_->az[count()] * 0.001);
			imp_->z[count()] = (imp_->z[count() - 1] + imp_->vz[count()] * 0.001);
		};

		auto obtainU = [&](double Force)
		{
			imp_->au[count()] = ((Force - (Iy - Iz) * imp_->vv[count() - 1] * imp_->vw[count() - 1]) / Ix);
			imp_->vu[count()] = (imp_->vu[count() - 1] + imp_->au[count()] * 0.001);
			imp_->u[count()] = (imp_->u[count() - 1] + imp_->vu[count()] * 0.001);
		};
		auto obtainV = [&](double Force)
		{
			imp_->av[count()] = ((Force - (Iz - Ix) * imp_->vw[count() - 1] * imp_->vu[count() - 1]) / Iy);
			imp_->vv[count()] = (imp_->vv[count() - 1] + imp_->av[count()] * 0.001);
			imp_->v[count()] = (imp_->v[count() - 1] + imp_->vv[count()] * 0.001);
		};
		auto obtainW = [&](double Force)
		{
			imp_->aw[count()] = ((Force - (Ix - Iy) * imp_->vu[count() - 1] * imp_->vv[count() - 1]) / Iz);
			imp_->vw[count()] = (imp_->vw[count() - 1] + imp_->aw[count()] * 0.001);
			imp_->w[count()] = (imp_->w[count() - 1] + imp_->vw[count()] * 0.001);
		};

		auto isSave = [&](double Position[6])
		{
			if ((Position[0] >= saveXYZ[0] && Position[0] <= saveXYZ[1]) &&
				(Position[1] >= saveXYZ[2] && Position[1] <= saveXYZ[3]) &&
				(Position[2] >= saveXYZ[4] && Position[2] <= saveXYZ[5]))
			{
				savetype = 0;
			}
			else if ((Position[0] > saveXYZ[1] && Position[0] < mmXYZ[1]) || (Position[0] < saveXYZ[0] && Position[0] > mmXYZ[0]) ||
					 (Position[1] > saveXYZ[3] && Position[1] < mmXYZ[3]) || (Position[1] < saveXYZ[2] && Position[1] > mmXYZ[2]) ||
					 (Position[2] > saveXYZ[5] && Position[2] < mmXYZ[5]) || (Position[2] < saveXYZ[4] && Position[2] > mmXYZ[4]))
			{
				savetype = 1;
			}
			else if (Position[0] <= mmXYZ[0] || Position[0] >= mmXYZ[1] ||
					 Position[1] <= mmXYZ[2] || Position[1] >= mmXYZ[3] ||
					 Position[2] <= mmXYZ[4] || Position[2] >= mmXYZ[5])
			{
				savetype = 2;
			}
		};

		auto isvSave = [&](double velocity[3])
		{
			for (int i = 0; i < 3; i++)
			{
				if (velocity[i] > 0.05)
				{
					mout() << "v is too high." << std::endl;
					isvwSave = 1;
				}
			}
		};

		auto iswSave = [&](double omega[3])
		{
			for (int i = 0; i < 3; i++)
			{
				if (omega[i] > 0.25)
				{
					mout() << "w is too high." << std::endl;
					isvwSave = 1;
				}
			}
		};

		auto getForceData = [&](double *data_, int m_, bool init_)
		{
			int raw_force[6]{0};

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

		auto forceFilter = [&](double *actual_force_, double *filtered_force_, int m_)
		{
			if (m_ == 0)
			{
				for (int i = 0; i < 6; i++)
				{
					imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
					imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

					filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
				}
			}
			else if (m_ == 1)
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
				mout() << "Wrong Filter!" << std::endl;
			}
		};

		auto forceTransform = [&](double *actual_force_, double *transform_force_, int m_)
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

		eeA1.getP(currentPos);

		if (count() == 1)
		{
			saveXYZ[0] = currentPos[0] - 0.15;
			saveXYZ[1] = currentPos[0] + 0.15;
			saveXYZ[2] = currentPos[1] - 0.15;
			saveXYZ[3] = currentPos[1] + 0.15;
			saveXYZ[4] = currentPos[2] - 0.15;
			saveXYZ[5] = currentPos[2] + 0.15;
			mmXYZ[0] = currentPos[0] - 0.2;
			mmXYZ[1] = currentPos[0] + 0.2;
			mmXYZ[2] = currentPos[1] - 0.2;
			mmXYZ[3] = currentPos[1] + 0.2;
			mmXYZ[4] = currentPos[2] - 0.2;
			mmXYZ[5] = currentPos[2] + 0.2;

			for(int i = 0; i < 6 ; i++){
				mout() << saveXYZ[i] << "  " << mmXYZ[i] << std::endl;
			}
		};

		isSave(currentPos);

		if (savetype == 0)
		{
			imp_->mycount1 = count();

			/* if(count() == 1){
				mout() << "savetype = " << savetype << std::endl;
				mout() << "currentpos " << currentPos[0] << "  " << currentPos[1] << "  " << currentPos[2] << "  "
					   << currentPos[3] << "  " << currentPos[4] << "  " << currentPos[5] << "  " << std::endl;
				imp_->x[0] = currentPos[0];
				imp_->y[0] = currentPos[1];
				imp_->z[0] = currentPos[2];
				imp_->u[0] = currentPos[3];
				imp_->v[0] = currentPos[4];
				imp_->w[0] = currentPos[5];
			}

			if (count() < 500)
			{
				currentForce[0] = 5;
				currentForce[1] = 7;
				currentForce[2] = 0;
				currentForce[3] = 0.2;
				currentForce[4] = 0.3;
				currentForce[5] = 0;
			}
			else
			{
				currentForce[0] = 0;
				currentForce[1] = 0;
				currentForce[2] = 0;
				currentForce[3] = 0;
				currentForce[4] = 0;
				currentForce[5] = 0;
			} */

			if (!imp_->init && !imp_->contact_check)
			{
				getForceData(imp_->arm1_init_force, 0, imp_->init);
				getForceData(imp_->arm2_init_force, 1, imp_->init);
				// master()->logFileRawName("ForceDrag");
				imp_->init = true;

				mout() << "savetype = " << savetype << std::endl;
				mout() << "currentpos " << currentPos[0] << "  " << currentPos[1] << "  " << currentPos[2] << "  "
					   << currentPos[3] << "  " << currentPos[4] << "  " << currentPos[0] << "  " << std::endl;
				imp_->x[0] = currentPos[0];
				imp_->y[0] = currentPos[1];
				imp_->z[0] = currentPos[2];
				imp_->u[0] = currentPos[3];
				imp_->v[0] = currentPos[4];
				imp_->w[0] = currentPos[5];
			}
			else if (imp_->init && !imp_->contact_check)
			{
				double raw_force_checker[12]{0};
				double comp_force_checker[12]{0};
				double force_checker[12]{0};

				double a1_pm[16]{0};
				double a2_pm[16]{0};

				eeA1.getMpm(a1_pm);
				eeA2.getMpm(a2_pm);

				// Arm1
				getForceData(raw_force_checker, 0, imp_->init);
				gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
				// Arm2
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
				eeA1.getP(arm1_current_pos);
				eeA1.getMpm(arm1_current_pm);

				// eeA2.getP(arm2_current_pos);
				// eeA2.getMpm(arm2_current_pm);

				// Force Comp, Filtered, Transform
				getForceData(arm1_actual_force, 0, imp_->init);
				gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

				// getForceData(arm2_actual_force, 1, imp_->init);
				// gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

				for (size_t i = 0; i < 6; i++)
				{
					arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
					// arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
				}

				forceFilter(arm1_comp_force, arm1_filtered_force, 0);
				// forceFilter(arm2_comp_force, arm2_filtered_force, 1);

				// forceDeadZone(arm1_filtered_force, arm1_dead_zone);
				// forceDeadZone(arm2_filtered_force, arm2_dead_zone);

				forceTransform(arm1_filtered_force, arm1_transform_force, 0);
				// forceTransform(arm2_filtered_force, arm2_transform_force, 1);

				for (size_t i = 0; i < 6; i++)
				{
					currentForce[i] = arm1_transform_force[i];
					// arm2_final_force[i] = arm2_transform_force[i] - imp_->arm2_start_force[i];
				}
			}

			obtainX(currentForce[0]);
			obtainY(currentForce[1]);
			obtainZ(currentForce[2]);

			obtainU(currentForce[3]);
			obtainV(currentForce[4]);
			obtainW(currentForce[5]);

			currentv[0] = imp_->vx[count()];
			currentv[1] = imp_->vy[count()];
			currentv[2] = imp_->vz[count()];

			currentw[0] = imp_->vu[count()];
			currentw[1] = imp_->vv[count()];
			currentw[2] = imp_->vw[count()];

			isvSave(currentv);
			iswSave(currentw);

			if (isvwSave == 1)
			{
				return 0;
			}

			targetPos[0] = imp_->x[count()];
			targetPos[1] = imp_->y[count()];
			targetPos[2] = imp_->z[count()];

			domega[0] = imp_->u[count()] - imp_->u[count() - 1];
			domega[1] = imp_->v[count()] - imp_->v[count() - 1];
			domega[2] = imp_->w[count()] - imp_->w[count() - 1];

			aris::dynamic::s_ra2rm(domega, drm);
			aris::dynamic::s_re2rm(currentPos + 3, currentrm, "321");
			aris::dynamic::s_mm(3, 3, 3, drm, currentrm, targetrm);
			aris::dynamic::s_rm2re(targetrm, targetPos + 3, "321");

			arm1.setOutputPos(targetPos);

			if (arm1.inverseKinematics())
			{
				mout() << "Error " << count() << std::endl;
				mout() << "count =  " << count() << '\n'
					   << " x = " << imp_->x[count()] << " y = " << imp_->y[count()] << " z = " << imp_->z[count()] << '\n'
					   << " vx = " << imp_->vx[count()] << " vy = " << imp_->vy[count()] << " vz = " << imp_->vz[count()] << '\n'
					   << " ax = " << imp_->ax[count()] << " ay = " << imp_->ay[count()] << " az = " << imp_->az[count()] << std::endl;
				// throw std::runtime_error("Inverse Kinematics Position Failed!");
				return 0;
			}

			arm1.getInputPos(targetAngle);

			for (int i = 0; i < 6; i++)
			{
				controller()->motorPool()[i].setTargetPos(targetAngle[i]);
			}
		}
		else if (savetype == 1)
		{
			imp_->mycount2++;
			if (imp_->mycount2 == 1) // 确定finalpos
			{
				mout() << "count = " << count() << std::endl;
				mout() << "savetype = " << savetype << std::endl;
				if (imp_->vx[imp_->mycount1] < 0)
				{
					imp_->finalPos[0] = mmXYZ[0];
				}
				if (imp_->vx[imp_->mycount1] >= 0)
				{
					imp_->finalPos[0] = mmXYZ[1];
				}
				if (imp_->vy[imp_->mycount1] < 0)
				{
					imp_->finalPos[1] = mmXYZ[2];
				}
				if (imp_->vy[imp_->mycount1] >= 0)
				{
					imp_->finalPos[1] = mmXYZ[3];
				}
				if (imp_->vz[imp_->mycount1] < 0)
				{
					imp_->finalPos[2] = mmXYZ[4];
				}
				if (imp_->vz[imp_->mycount1] >= 0)
				{
					imp_->finalPos[2] = mmXYZ[5];
				}
				imp_->finalPos[3] = currentPos[3];
				imp_->finalPos[4] = currentPos[4];
				imp_->finalPos[5] = currentPos[5];
				mout() << "finailpos = " << imp_->finalPos[0] << "," << imp_->finalPos[1] << "," << imp_->finalPos[2] << std::endl;
			}
			for (int i = 0; i < 3; i++)
			{
				double type2v = 0;
				if (i == 0)
				{
					type2v = imp_->vx[imp_->mycount1];
				}
				if (i == 1)
				{
					type2v = imp_->vy[imp_->mycount1];
				}
				if (i == 2)
				{
					type2v = imp_->vz[imp_->mycount1];
				}

				if (abs(currentPos[i] - imp_->finalPos[i]) < 0.001)
				{
					targetPos[i] = imp_->finalPos[i];
				}
				else
				{
					if (currentPos[i] < imp_->finalPos[i])
					{
						targetPos[i] = (currentPos[i] + type2v * 0.001);
					}
					else if (currentPos[i] > imp_->finalPos[i])
					{
						targetPos[i] = (currentPos[i] - type2v * 0.001);
					}
				}
			}
			targetPos[3] = currentPos[3];
			targetPos[4] = currentPos[4];
			targetPos[5] = currentPos[5];

			arm1.setOutputPos(targetPos);

			if (arm1.inverseKinematics())
			{
				mout() << "Error " << count() << std::endl;
				mout() << "count =  " << count() << '\n'
					   << " x = " << imp_->x[count()] << " y = " << imp_->y[count()] << " z = " << imp_->z[count()] << '\n'
					   << " vx = " << imp_->vx[count()] << " vy = " << imp_->vy[count()] << " vz = " << imp_->vz[count()] << '\n'
					   << " ax = " << imp_->ax[count()] << " ay = " << imp_->ay[count()] << " az = " << imp_->az[count()] << std::endl;
				// throw std::runtime_error("Inverse Kinematics Position Failed!");
				return 0;
			}

			arm1.getInputPos(targetAngle);

			for (int i = 0; i < 6; i++)
			{
				controller()->motorPool()[i].setTargetPos(targetAngle[i]);
			}
		}
		else if (savetype == 2)
		{
			mout() << "savetype = 2, Out of range" << std::endl;
			return 0;
		}

		if (count() % 100 == 0)
		{
			mout() << "count =  " << count() << '\n'
				   << " x = " << imp_->x[count()] << " y = " << imp_->y[count()] << " z = " << imp_->z[count()] << '\n'
				   << " vx = " << imp_->vx[count()] << " vy = " << imp_->vy[count()] << " vz = " << imp_->vz[count()] << '\n'
				   << " ax = " << imp_->ax[count()] << " ay = " << imp_->ay[count()] << " az = " << imp_->az[count()] << std::endl;

			mout() << " u = " << imp_->u[count()] << " v = " << imp_->v[count()] << " w = " << imp_->w[count()] << '\n'
				   << " vu = " << imp_->vu[count()] << " vv = " << imp_->vv[count()] << " vw = " << imp_->vw[count()] << '\n'
				   << " au = " << imp_->au[count()] << " av = " << imp_->av[count()] << " aw = " << imp_->aw[count()] << std::endl;

			mout() << "targetpos " << targetPos[0] << "  " << targetPos[1] << "  " << targetPos[2] << "  "
				   << targetPos[3] << "  " << targetPos[4] << "  " << targetPos[0] << "  " << std::endl; 

			/* mout() << "currentpos " << currentPos[0] << "  " << currentPos[1] << "  " << currentPos[2] << "  "
				   << currentPos[3] << "  " << currentPos[4] << "  " << currentPos[5] << "  " << std::endl; */

		}

		return 8000 - count();
	}
	ZeroG::ZeroG(const std::string &name)
	{
		aris::core::fromXmlString(command(),
								  "<Command name=\"zerog\">"
								  "</Command>");
	}
	ZeroG::~ZeroG() = default;

    KAANH_DEFINE_BIG_FOUR_CPP(ZeroG)

    ARIS_REGISTRATION {
    aris::core::class_<ZeroG>("ZeroG")
        .inherit<aris::plan::Plan>();

    }


}