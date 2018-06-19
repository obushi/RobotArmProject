//
//  trajectory_generator.hpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#pragma once

#include <functional>
#include <iostream>
#include <array>
#include <vector>
#include <string>
#include <sstream>
#include "trajectory.hpp"

namespace robot_arm_project {
    
    class Trajectory {
        
        double init_angle;
        double init_velocity;
        double goal_angle;
        double goal_velocity;
        double duration; // [sec]
    public:
        Trajectory(double init_angle, double init_velocity, double goal_angle, double goal_velocity, double duration);
        std::string ToExpression();
        std::function<double(double)> ToFunction();
//        std::function<std::array<double, 6>(double)> GetFunctions(std::array<double, 6> init_pos, std::array<double, 6> init_vel, std::array<double, 6> goal_pos, std::array<double, 6> goal_vel, double duration);
    };
}
