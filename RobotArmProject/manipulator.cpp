//
//  manipulator.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "manipulator.hpp"

namespace robot_arm_project {
    
    const TransformMatrix & Manipulator::GetTransformMatrix() const {
        return mat;
    }
    
    void Manipulator::SetTransformMatrix(const TransformMatrix & new_matrix) {
        mat = new_matrix;
    }
    
    const JointAngle & Manipulator::GetJointAngles() const {
        return joint_angle;
    }
    
    void Manipulator::SetJointAngles(const JointAngle & new_angles) {
        joint_angle = new_angles;
    }
    
    const TransformMatrix Manipulator::SolveKinematics(double p1, double p2, double p3, double p4, double p5, double p6) const {
        return TransformMatrix(
        /* r11 */ cos(p1)*(cos(p2+p3)*(cos(p4)*cos(p5)*cos(p6)-sin(p4)*sin(p6))-sin(p2+p3)*sin(p5)*cos(p6))-sin(p1)*(sin(p4)*cos(p5)*cos(p6)+cos(p4)*sin(p6)),
        /* r21 */ sin(p1)*(cos(p2+p3)*(cos(p4)*cos(p5)*cos(p6)-sin(p4)*sin(p6))-sin(p2+p3)*sin(p5)*cos(p6))+cos(p1)*(sin(p4)*cos(p5)*cos(p6)+cos(p4)*sin(p6)),
        /* r31 */ -sin(p2+p3)*(cos(p4)*cos(p5)*cos(p6)-sin(p4)*sin(p6))-cos(p2+p3)*sin(p5)*cos(p6),
        /* r12 */ cos(p1)*(-cos(p2+p3)*(cos(p4)*cos(p5)*sin(p6)+sin(p4)*cos(p6))+sin(p2+p3)*sin(p5)*sin(p6))-sin(p1)*(-sin(p4)*cos(p5)*sin(p6)+cos(p4)*cos(p6)),
        /* r22 */ sin(p1)*(-cos(p2+p3)*(cos(p4)*cos(p5)*sin(p6)+sin(p4)*cos(p6))+sin(p2+p3)*sin(p5)*sin(p6))+cos(p1)*(-sin(p4)*cos(p5)*sin(p6)+cos(p4)*cos(p6)),
        /* r32 */ sin(p2+p3)*(cos(p4)*cos(p5)*sin(p6)+sin(p4)*cos(p6))+cos(p2+p3)*sin(p5)*sin(p6),
        /* r13 */ cos(p1)*(cos(p2+p3)*cos(p4)*sin(p5)+sin(p2+p3)*cos(p5))-sin(p1)*sin(p4)*sin(p5),
        /* r23 */ sin(p1)*(cos(p2+p3)*cos(p4)*sin(p5)+sin(p2+p3)*cos(p5))+cos(p1)*sin(p4)*sin(p5),
        /* r33 */ -sin(p2+p3)*cos(p4)*sin(p5)+cos(p2+p3)*cos(p5),
        /*  px */ cos(p1)*(sin(p2+p3)*d4+sin(p2)*a2+a1),
        /*  py */ sin(p1)*(sin(p2+p3)*d4+sin(p2)*a2+a1),
        /*  pz */ cos(p2+p3)*d4+cos(p2)*a2
        );
    }
    
    const TransformMatrix Manipulator::SolveKinematicsDegree(double p1, double p2, double p3, double p4, double p5, double p6) const {
        return SolveKinematics(Deg2Rad(p1), Deg2Rad(p2), Deg2Rad(p3), Deg2Rad(p4), Deg2Rad(p5), Deg2Rad(p6));
    }
    
    const JointAngle Manipulator::SolveInverseKinematics(const TransformMatrix & mat) const {
        double p1 = atan2(mat.py, mat.px);
        double p3 = acos(((cos(p1)*mat.px+sin(p1)*mat.py-a1)*(cos(p1)*mat.px+sin(p1)*mat.py-a1)+mat.pz*mat.pz-a2*a2-d4*d4)/(2*a2*d4));
        if (std::abs(joint_angle.p3 - p3) > std::abs(joint_angle.p3 + p3)) {
            p3 = -p3;
        }
        double p2 = atan2(a2*sin(p3)*mat.pz+(cos(p1)*mat.px+sin(p1)*mat.py-a1)*(a2*cos(p3)+d4),(a2*cos(p3)+d4)*mat.pz-a2*sin(p3)*(cos(p1)*mat.px+sin(p1)*mat.py-a1))-p3;
        double p4 = atan2(-mat.r13*sin(p1)+mat.r23*cos(p1),mat.r13*cos(p1)*cos(p2+p3)+mat.r23*sin(p1)*cos(p2+p3)-mat.r33*sin(p2+p3));
        double p5 = atan2(mat.r13*(cos(p1)*cos(p2+p3)*cos(p4)-sin(p1)*sin(p4))+mat.r23*(sin(p1)*cos(p2+p3)*cos(p4)+cos(p1)*sin(p4))+mat.r33*(-sin(p2+p3)*cos(p4)),mat.r13*(cos(p1)*sin(p2+p3))+mat.r23*(sin(p1)*sin(p2+p3))+mat.r33*cos(p2+p3));
        double p6 = atan2(-mat.r11*(cos(p1)*cos(p2+p3)*sin(p4)+sin(p1)*cos(p4))-mat.r21*(sin(p1)*cos(p2+p3)*sin(p4)-cos(p1)*cos(p4))+mat.r31*sin(p2+p3)*sin(p4),mat.r11*(cos(p5)*(cos(p1)*cos(p2+p3)*cos(p4)-sin(p1)*sin(p4))-cos(p1)*sin(p2+p3)*sin(p5))+mat.r21*(cos(p5)*(sin(p1)*cos(p2+p3)*cos(p4)+cos(p1)*sin(p4))-sin(p1)*sin(p2+p3)*sin(p5))+mat.r31*(-sin(p2+p3)*cos(p4)*cos(p5)-cos(p2+p3)*sin(p5)));
        return JointAngle(p1, p2, p3, p4, p5, p6);
    }
    
    void Manipulator::Simulate(double duration_in_second, double dt_in_second) {
        if (trajectories.size() <= 0) {
            return;
        }
        
        for (double t = 0; t <= duration_in_second; t += dt_in_second) {
            pd_controller.Compute(dt_in_second, <#std::array<double, 6> observed#>, <#std::array<double, 6> target#>)
        }
    }
    
    void Manipulator::RenderTrajectory() {
        
    }
    
    void Manipulator::Print() const {
        mat.Print();
        joint_angle.Print();
    }
    
    void Manipulator::PrintDegree() const {
        mat.Print();
        joint_angle.PrintDegree();
    }
}
