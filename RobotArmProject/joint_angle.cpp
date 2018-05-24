//
//  joint_angle.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "joint_angle.hpp"

namespace robot_arm_project {
    
    JointAngle::JointAngle(double p1, double p2, double p3, double p4, double p5, double p6):
    p1(p1), p2(p2), p3(p3), p4(p4), p5(p5), p6(p6) {}
    double p1, p2, p3, p4, p5, p6;
    
    void JointAngle::Print() const {
        std::cout << "Joint Angles:" << std::endl;
        std::cout << "p1[rad] : " << p1 << std::endl;
        std::cout << "p2[rad] : " << p2 << std::endl;
        std::cout << "p3[rad] : " << p3 << std::endl;
        std::cout << "p4[rad] : " << p4 << std::endl;
        std::cout << "p5[rad] : " << p5 << std::endl;
        std::cout << "p6[rad] : " << p6 << std::endl;
    }
    
    void JointAngle::PrintDegree() const {
        std::cout << "Joint Angles:" << std::endl;
        std::cout << "p1[deg] : " << Rad2Deg(p1) << std::endl;
        std::cout << "p2[deg] : " << Rad2Deg(p2) << std::endl;
        std::cout << "p3[deg] : " << Rad2Deg(p3) << std::endl;
        std::cout << "p4[deg] : " << Rad2Deg(p4) << std::endl;
        std::cout << "p5[deg] : " << Rad2Deg(p5) << std::endl;
        std::cout << "p6[deg] : " << Rad2Deg(p6) << std::endl;
    }
} // namespace robot_arm_project
