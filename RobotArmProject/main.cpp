//
//  main.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/04/19.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include <iostream>
#include <cmath>

namespace robot_arm_project {

inline double Deg2Rad(double deg){ return deg / 180.0 * M_PI;}
inline double Rad2Deg(double rad){ return rad * 180.0 / M_PI;}
    
struct TransformMatrix {
    TransformMatrix(double r11=1.0, double r21=0.0, double r31=0.0, double r12=0.0, double r22=1.0, double r32=0.0, double r13=0.0, double r23=0.0, double r33=1.0, double px=0.0, double py=0.0, double pz=0.0):
    r11(r11), r21(r21), r31(r31), r12(r12), r22(r22), r32(r32), r13(r13), r23(r23), r33(r33), px(px), py(py), pz(pz) {}
    double r11, r21, r31, r12, r22, r32, r13, r23, r33, px, py, pz;

    void Print() const {
        std::cout << "Transform Matrix:" << std::endl;
        std::cout << r11 << "\t" << r12 << "\t" << r13 << "\t" << px << std::endl;
        std::cout << r21 << "\t" << r22 << "\t" << r23 << "\t" << py << std::endl;
        std::cout << r31 << "\t" << r32 << "\t" << r33 << "\t" << pz << std::endl;
        std::cout <<   0 << "\t" <<   0 << "\t" <<   0 << "\t" <<  1 << std::endl;
    }
};
    
struct JointAngle {
    JointAngle(double p1=0.0, double p2=0.0, double p3=0.0, double p4=0.0, double p5=0.0, double p6=0.0):
    p1(p1), p2(p2), p3(p3), p4(p4), p5(p5), p6(p6) {}
    double p1, p2, p3, p4, p5, p6;
    
    void Print() const {
        std::cout << "Joint Angles:" << std::endl;
        std::cout << "p1[rad] : " << p1 << std::endl;
        std::cout << "p2[rad] : " << p2 << std::endl;
        std::cout << "p3[rad] : " << p3 << std::endl;
        std::cout << "p4[rad] : " << p4 << std::endl;
        std::cout << "p5[rad] : " << p5 << std::endl;
        std::cout << "p6[rad] : " << p6 << std::endl;
    }
    
    void PrintDegree() const {
        std::cout << "Joint Angles:" << std::endl;
        std::cout << "p1[deg] : " << robot_arm_project::Rad2Deg(p1) << std::endl;
        std::cout << "p2[deg] : " << robot_arm_project::Rad2Deg(p2) << std::endl;
        std::cout << "p3[deg] : " << robot_arm_project::Rad2Deg(p3) << std::endl;
        std::cout << "p4[deg] : " << robot_arm_project::Rad2Deg(p4) << std::endl;
        std::cout << "p5[deg] : " << robot_arm_project::Rad2Deg(p5) << std::endl;
        std::cout << "p6[deg] : " << robot_arm_project::Rad2Deg(p6) << std::endl;
    }
};

class Manipulator {
    constexpr static double a1 = 20;
    constexpr static double a2 = 165;
    constexpr static double d3 = 0;
    constexpr static double d4 = 165;
    TransformMatrix mat;
    JointAngle joint_angle;
    bool is_busy = false;
    
public:
    const TransformMatrix & GetTransformMatrix() const {
        return mat;
    }
    
    void SetTransformMatrix(const TransformMatrix & new_matrix) {
        mat = new_matrix;
    }
    
    const JointAngle & GetJointAngles() const {
        return joint_angle;
    }
    
    void SetJointAngles(const JointAngle & new_angles) {
        joint_angle = new_angles;
    }
    
    const TransformMatrix SolveKinematics(double p1, double p2, double p3, double p4, double p5, double p6) const {
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
    
    const TransformMatrix SolveKinematicsDegree(double p1, double p2, double p3, double p4, double p5, double p6) const {
        return SolveKinematics(Deg2Rad(p1), Deg2Rad(p2), Deg2Rad(p3), Deg2Rad(p4), Deg2Rad(p5), Deg2Rad(p6));
    }
    
    const JointAngle SolveInverseKinematics(const TransformMatrix & mat) const {
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
    
    void Print() const {
        mat.Print();
        joint_angle.Print();
    }
    
    void PrintDegree() const {
        mat.Print();
        joint_angle.PrintDegree();
    }
};
} // namespace robot_arm_project

int main(int argc, const char * argv[]) {
    robot_arm_project::Manipulator manipulator;
    
    // 運動学の計算・変換行列の代入
    manipulator.SetTransformMatrix(manipulator.SolveKinematicsDegree(90,90,0,0,0,0));
    
    // 変換行列を元に逆運動学の計算・各関節の角度を求める
    manipulator.SetJointAngles(manipulator.SolveInverseKinematics(manipulator.GetTransformMatrix()));
    
    // 運動学->逆運動学の順に計算すると元の角度が得られる
    manipulator.PrintDegree();
    
    return 0;
}
