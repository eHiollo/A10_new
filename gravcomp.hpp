class GravComp
{
private:

    auto getTempFMatrix(double force_data_[6], double temp_[18]) -> void;
    auto getTempRMatrix(double pose_matrix_[9], double temp_[18]) -> void;

public:

    auto getTorqueVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double torque_vector_[9]) -> void;
    auto getForceVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double force_vector_[9]) -> void;
    auto getFMatrix(double force_data1_[6], double force_data2_[6], double force_data3_[6], double f_matrix_[54]) -> void;
    auto getRMatrix(double pose_matrix1_[9], double pose_matrix2_[9], double pose_matrix3_[9], double r_matrix_[54]) -> void;
    auto getPLMatrix(double f_r_matrix_[54], double torque_force_data_[9], double P_L[6]) -> void;
    auto getCompFT(double current_pose_[16], double L_[6], double P_[6], double comp_f_[6]) -> void;
    auto getInverseRm(double rotation_matrix_[9], double inverse_rot_[9]) -> void;


    auto savePLVector(const double P1_[6], const double L1_[6], const double P2_[6], const double L2_[6]) -> void;
    auto loadPLVector(double P1_[6], double L1_[6], double P2_[6], double L2_[6]) -> void;

    auto saveInitForce(const double arm1_init_force_[6], const double arm2_init_force_[6]) -> void;
    auto loadInitForce(double arm1_init_force_[6], double arm2_init_force_[6]) -> void;

};
