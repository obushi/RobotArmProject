//
//  main.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/04/19.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "manipulator.hpp"
#include "grapher.hpp"
#include "trajectory.hpp"

int main(int argc, const char * argv[]) {
    
    robot_arm_project::Manipulator moto_mini;
    
    // 運動学の計算・変換行列の代入
    moto_mini.SetTransformMatrix(moto_mini.SolveKinematicsDegree(90,90,0,0,0,0));
    
    // 変換行列を元に逆運動学の計算・各関節の角度を求める
    moto_mini.SetJointAngles(moto_mini.SolveInverseKinematics(moto_mini.GetTransformMatrix()));
    
    // 運動学->逆運動学の順に計算すると元の角度が得られる
    moto_mini.PrintDegree();
    
    robot_arm_project::TransformMatrix mat(90, 10, 10, 0, 0, 0);
    moto_mini.SolveInverseKinematics(mat);
    
    robot_arm_project::Trajectory tj(0, 0, 10, 0, 3);
    
    robot_arm_project::Grapher gnuplot;
    gnuplot.SetRangeX(0, 3);
    gnuplot.Plot(tj.ToExpression());
    
    moto_mini.Simulate(10.0, 0.001);
    
    return 0;
}
