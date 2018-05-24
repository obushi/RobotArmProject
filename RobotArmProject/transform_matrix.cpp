//
//  transform_matrix.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "transform_matrix.hpp"

namespace robot_arm_project {
    
    TransformMatrix::TransformMatrix(double r11, double r21, double r31, double r12, double r22, double r32, double r13, double r23, double r33, double px, double py, double pz):
    r11(r11), r21(r21), r31(r31), r12(r12), r22(r22), r32(r32), r13(r13), r23(r23), r33(r33), px(px), py(py), pz(pz) {}
    double r11, r21, r31, r12, r22, r32, r13, r23, r33, px, py, pz;
    
    void TransformMatrix::Print() const {
        std::cout << "Transform Matrix:" << std::endl;
        std::cout << r11 << "\t" << r12 << "\t" << r13 << "\t" << px << std::endl;
        std::cout << r21 << "\t" << r22 << "\t" << r23 << "\t" << py << std::endl;
        std::cout << r31 << "\t" << r32 << "\t" << r33 << "\t" << pz << std::endl;
        std::cout <<   0 << "\t" <<   0 << "\t" <<   0 << "\t" <<  1 << std::endl;
    }
}
