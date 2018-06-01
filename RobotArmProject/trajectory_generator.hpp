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
#include <string>
#include <sstream>

namespace robot_arm_project {
    
    class TrajectoryGenerator {
    public:
        std::string CalclateTrajectory(double init_angle, double init_vel, double goal_angle, double goal_vel, double duration);
        std::function<double(double)> CalclateTrajectoryFunction(double init_pos, double init_vel, double goal_pos, double goal_vel, double duration);
        std::function<std::array<double, 6>(double)> CalclateTrajectoryFunctions(std::array<double, 6> init_pos, std::array<double, 6> init_vel, std::array<double, 6> goal_pos, std::array<double, 6> goal_vel, double duration);
    };
}
