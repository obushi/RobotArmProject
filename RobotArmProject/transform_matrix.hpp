//
//  transform_matrix.hpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#pragma once

#include <iostream>

namespace robot_arm_project {
    
    struct TransformMatrix {
        TransformMatrix(double r11=1.0, double r21=0.0, double r31=0.0, double r12=0.0, double r22=1.0, double r32=0.0, double r13=0.0, double r23=0.0, double r33=1.0, double px=0.0, double py=0.0, double pz=0.0);
        double r11, r21, r31, r12, r22, r32, r13, r23, r33, px, py, pz;
        void Print() const;
    };
}
