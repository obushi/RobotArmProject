//
//  manipulator.hpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#pragma once

#include "transform_matrix.hpp"
#include "joint_angle.hpp"

namespace robot_arm_project {
    
    class Manipulator {
        constexpr static double a1 = 20;
        constexpr static double a2 = 165;
        constexpr static double d3 = 0;
        constexpr static double d4 = 165;
        TransformMatrix mat;
        JointAngle joint_angle;
        
    public:
        const TransformMatrix & GetTransformMatrix() const;
        void SetTransformMatrix(const TransformMatrix & new_matrix);
        const JointAngle & GetJointAngles() const;
        void SetJointAngles(const JointAngle & new_angles);
        const TransformMatrix SolveKinematics(double p1, double p2, double p3, double p4, double p5, double p6) const;
        const TransformMatrix SolveKinematicsDegree(double p1, double p2, double p3, double p4, double p5, double p6) const;
        const JointAngle SolveInverseKinematics(const TransformMatrix & mat) const;
        void Print() const;
        void PrintDegree() const;
    };
}
