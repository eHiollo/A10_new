#include "movel.hpp"
#include <cmath>
#include <iostream>

using namespace std;
const double PI = 3.141592653589793;

namespace my_cmd
{   
    struct MoveLL::Imp {
			//Flag
			//bool init = false;
			//Arm1
			double arm1_p[6]{ 0 };
			//Arm2
			double arm2_p[6]{ 0 };
			//Desired Pos, Vel,
			double arm1_p_d[6]{ 0 };
			double arm2_p_d[6]{ 0 };
			double v_d[6] {0};
			//Current Vel
			double v_c[6]{ 0 };
			//Input Parameter
			double v = 0.05;
			//double target_pos[6]={0};
			//Test
			double actual_v[6]{ 0 };
			//Switch Model
			//m=0 -> Arm1
			//m=1 -> Arm2
			int m_=0;
			//速度控制数量
			double move_n;
			double xn=0;
			double yn=0;
			double zn=0;
			//临时变量 位置点输入
			double arm1_p_dump[6]{0};
			double dis_temp=0;

	};


    auto MoveLL::prepareNrt()->void 
    {
		std::cout << "[Process] Start MoveLL prepareNrt" << std::endl;
    	for (auto& m : motorOptions()) {
		m = aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;}

    }

	auto MoveLL::executeRT()->int
	{   
		// aris中所有初始化的模型（xml中定义）都存储在modelBase()中，ModelBase是所有模型的基类，有一些针对具体模型的操作无法直接对ModelBase使用，因此需要进行类型转换
		// 首先将modelBase转换为MultiModel类型，MultiModel是一个包含多个子模型的模型类
		// 可以直接对MultiModel进行正逆运动学等操作
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		// 由于本例子中定义的是包含两个机械臂的双臂机器人，因此可以通过subModels()方法获取到这两个子模型
		// at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		// at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);
		// 再将其转换为具体的Model类型，以便进行更具体的操作
		auto&  model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);
		// 获取末端执行器指针
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		//获取当前位置
		model_a1.getOutputPos(imp_->arm1_p);


		//只执行一次 且参数传到imp_中
		if(count()==1){
			std::cout << "[Process] Start executeRT" << std::endl;
			std::cout <<"【DEBUG】初始位置为"<< imp_->arm1_p[0]<<" "<< imp_->arm1_p[1]<< " "<< imp_->arm1_p[2]<<std::endl;
			double current_angle[6]{0};
			
			// std::cout<<"【DEBUG】电机获取的电机初始位置为"<<std::endl;
			// for (int i = 0; i < 6; i++)
			// {
			// 	current_angle[i] = controller()->motorPool()[i].targetPos();
			// 	std::cout<<current_angle[i]<<"  ";
			// }
			// model_a1.setOutputPos(current_angle);
			model_a1.getOutputPos(imp_->arm1_p);
			std::cout <<"【DEBUG】根据电机信息正运动学的位置"<< imp_->arm1_p[0]<<" "<< imp_->arm1_p[1]<< " "<< imp_->arm1_p[2]<<std::endl;
			// //初始化机器人位置
			// imp_->arm1_p[0]=0;
			// imp_->arm1_p[1]=0.5;
			// imp_->arm1_p[2]=0.5;
			std::copy(imp_->arm1_p, imp_->arm1_p + 6, imp_->arm1_p_dump);

			model_a1.setOutputPos(imp_->arm1_p);
			if (model_a1.inverseKinematics()) std::cout << "【init robot pos】arm1 inverse failed" << std::endl;
			double arm1_input_pos[6]{0};
			model_a1.getInputPos(arm1_input_pos);
			for(int j = 0 ; j < 6; j++){
				controller()->motorPool()[j].setTargetPos(arm1_input_pos[j]);
			}


			//选择机械臂并读取参数	
			if(doubleParam("vel")!=0.005) imp_->v=doubleParam("vel");
			if(imp_->m_==0){
				//位置
				if(doubleParam("x")!=0.0001) imp_->arm1_p_d[0]=doubleParam("x");
				else imp_->arm1_p_d[0]=imp_->arm1_p[0];
				if(doubleParam("y")!=0.0001) imp_->arm1_p_d[1]=doubleParam("y");
				else imp_->arm1_p_d[1]=imp_->arm1_p[1];
				if(doubleParam("z")!=0.0001) imp_->arm1_p_d[2]=doubleParam("z");
				else imp_->arm1_p_d[2]=imp_->arm1_p[2];
			}else{
				//位置
				if(doubleParam("x")!=0.0001) imp_->arm2_p_d[0]=doubleParam("x");
				else imp_->arm2_p_d[0]=imp_->arm1_p[0];
				if(doubleParam("y")!=0.0001) imp_->arm2_p_d[1]=doubleParam("y");
				else imp_->arm2_p_d[1]=imp_->arm1_p[1];
				if(doubleParam("z")!=0.0001) imp_->arm2_p_d[2]=doubleParam("z");
				else imp_->arm2_p_d[2]=imp_->arm2_p[2];
		}
			std::cout << "【目标】目标位置为"<< imp_->arm1_p_d[0]<<" "<< imp_->arm1_p_d[1]<< " "<< imp_->arm1_p_d[2]<<std::endl;
			std::cout << "【目标】目标速度为"<< imp_->v<<std::endl;
			//控速参数计算		
			double dis=Dis(imp_->arm1_p_d,imp_->arm1_p);
			imp_->dis_temp = dis;
			imp_->move_n = dis/(0.002*imp_->v);
			if(imp_->move_n<1) imp_->move_n=1;
			std::cout << "【参数】一共需要"<<imp_->move_n<<"个步骤完成" << std::endl;	

			//计算每一步的增量
			 imp_->xn=(imp_->arm1_p_d[0]-imp_->arm1_p[0])/imp_->move_n; 
			 imp_->yn=(imp_->arm1_p_d[1]-imp_->arm1_p[1])/imp_->move_n;
			 imp_->zn=(imp_->arm1_p_d[2]-imp_->arm1_p[2])/imp_->move_n; 
			//  std::cout << "【增量xn：】"<< imp_->xn << std::endl;
			//  std::cout << "【增量yn：】"<< imp_->yn << std::endl;
			//  std::cout << "【增量zn：】"<< imp_->zn << std::endl;
			//return 0;
		}
		//更新当前位置
		imp_->arm1_p_dump[0]=imp_->arm1_p[0];
		imp_->arm1_p_dump[1]=imp_->arm1_p[1];
		imp_->arm1_p_dump[2]=imp_->arm1_p[2];
		//控制速度
		double dis=Dis(imp_->arm1_p_d,imp_->arm1_p);
		if(dis > 0.005) {
			//计算每一个运行点
			imp_->arm1_p_dump[0]+=imp_->xn;
			imp_->arm1_p_dump[1]+=imp_->yn;
			imp_->arm1_p_dump[2]+=imp_->zn;
			model_a1.setOutputPos(imp_->arm1_p_dump);
			if (model_a1.inverseKinematics() && count() % 500 ==0) {
                std::cout << "arm1 inverse failed, pos: "
                    << imp_->arm1_p_dump[0] << " "
                    << imp_->arm1_p_dump[1] << " "
                    << imp_->arm1_p_dump[2] << std::endl;
                // 跳过本次循环，等待下次
                return 0;
            }
			double arm1_input_pos[6]{0};
			model_a1.getInputPos(arm1_input_pos);

			for(int j = 0 ; j < 6; j++){
				controller()->motorPool()[j].setTargetPos(arm1_input_pos[j]);
			}

			if(count() % 500==0) {
				std::cout<<"【正在运动】距离目标点"<<Dis(imp_->arm1_p_d,imp_->arm1_p)<<std::endl;
				std::cout<<"【正在运动】第"<<count()<<"次进行完成"<<std::endl;}

			//距离没动就退出
			// if(count()!=1 && dis==imp_->dis_temp){
			// 	std::cout << "【警告】距离没有变化，运动失败，当前位置为"<< imp_->arm1_p[0]<<" "<< imp_->arm1_p[1]<< " "<< imp_->arm1_p[2]<<std::endl;
			// 	return 0;
			// }else{
			// 	imp_->dis_temp = dis;
			// }
		}	
		else{
			std::cout << "【完成】运动结束，当前位置为"<< imp_->arm1_p[0]<<" "<< imp_->arm1_p[1]<< " "<< imp_->arm1_p[2]<<std::endl;
			int printDis = Dis(imp_->arm1_p_d,imp_->arm1_p);
			return 0;
		} 
			if(count()==10000) return 0;
			return count();
	}
	MoveLL::MoveLL(const std::string& name)
	{   
        std::cout << "[DEBUG] MoveLL constructor called." << std::endl;

        aris::core::fromXmlString(command(),
            "<Command name=\"movell\">"
            //"    <GroupParam>"
            //"        <Param name=\"pos\" abbreviation=\"m\" default=\"0.1,0.2,0.3,0,0,0\"/>"
			//"		 <Param name=\"x\"   default=\"0.1\"/>"
			//"		 <Param name=\"y\"   default=\"0.2\"/>"
			//"		 <Param name=\"z\"   default=\"0.3\"/>"
            //"        <Param name=\"vel\" abbreviation=\"v\" default=\"0.2\"/>"
            //"    </GroupParam>"
            "</Command>");
	}
    MoveLL::~MoveLL() = default;
	

		// 计算六维数组中前三维的 XYZ 坐标之间的距离
	double MoveLL::Dis(const double arr1[6], const double arr2[6]) {
		double sum = 0.0;
		// 只计算前三维坐标的距离：X, Y, Z
		for (int i = 0; i < 3; i++) {
			sum += std::pow(arr2[i] - arr1[i], 2);  // (arr2[i] - arr1[i])^2
		}
		return std::sqrt(sum);  // 计算平方和后开方得到距离
	}

}

// ARIS_REGISTRATION{
// 	std::cout << "[DEBUG] Registering MoveLL..." << std::endl;
// 	aris::core::class_<my_cmd::MoveLL>("MoveLL")
// 		.inherit<aris::plan::Plan>();
// }

// <MoveLL name="plan">
//                 <Command __prop_name__="command" name="movell">
//                     <GroupParam name="group_param">
//                         <Param name="pos" abbreviation="p" default="0.0,0.0,0.0,0,0,0"/>
//                         <Param name="x" abbreviation="x" default="0.0001"/>
//                         <Param name="y" abbreviation="y" default="0.0001"/>
//                         <Param name="z" abbreviation="z" default="0.0001"/>
//                         <Param name="vel" abbreviation="v" default="0.05"/>
//                     </GroupParam>
//                 </Command>
//             </MoveLL>