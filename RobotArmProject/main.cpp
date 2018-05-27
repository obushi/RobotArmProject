//
//  main.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/04/19.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "manipulator.hpp"
#include "grapher.hpp"
#include "trajectory_generator.hpp"

int main(int argc, const char * argv[]) {
    
    robot_arm_project::Manipulator manipulator;
    
    // 運動学の計算・変換行列の代入
    manipulator.SetTransformMatrix(manipulator.SolveKinematicsDegree(90,90,0,0,0,0));
    
    // 変換行列を元に逆運動学の計算・各関節の角度を求める
    manipulator.SetJointAngles(manipulator.SolveInverseKinematics(manipulator.GetTransformMatrix()));
    
    // 運動学->逆運動学の順に計算すると元の角度が得られる
    manipulator.PrintDegree();
    
    robot_arm_project::Grapher gnuplot;
    gnuplot.Plot("cos(x)");
    
    robot_arm_project::TrajectoryGenerator tg;
    auto func = tg.CalclateTrajectoryFunction(0, 0, 10, 0, 3);
    std::cout << func(0) << std::endl;
    
    gnuplot.SetRangeX(0, 3);
    gnuplot.Plot(tg.CalclateTrajectory(0, 0, 10, 0, 3).c_str());
    
    return 0;
}
