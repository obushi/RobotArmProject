//
//  joint_angle.hpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#pragma once
#include <iostream>
#include <cmath>

namespace robot_arm_project {
    
    inline double Deg2Rad(double deg){ return deg / 180.0 * M_PI;}
    inline double Rad2Deg(double rad){ return rad * 180.0 / M_PI;}
    
    struct JointAngle {
        JointAngle(double p1=0.0, double p2=0.0, double p3=0.0, double p4=0.0, double p5=0.0, double p6=0.0);
        double p1, p2, p3, p4, p5, p6;
        void Print() const;
        void PrintDegree() const;
    };
} // namespace robot_arm_project
