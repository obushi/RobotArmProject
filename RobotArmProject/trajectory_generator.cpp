//
//  trajectory_generator.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "trajectory_generator.hpp"

namespace robot_arm_project {
    
    std::string TrajectoryGenerator::CalclateTrajectory(double init_angle, double init_vel, double goal_angle, double goal_vel, double duration) {
        double a0 = init_angle;
        double a1 = init_vel;
        double a2 = 3.0 / duration / duration * (goal_angle - init_angle) - 2.0 / duration * init_vel - goal_vel / duration;
        double a3 = - 2.0 / duration / duration / duration * (goal_angle - init_angle) + 1.0 / duration / duration * (goal_vel + init_vel);
        std::stringstream str;
        str << a3 << "*x**3+" << a2 << "*x**2+" << a1 << "*x+" << a0;
        return str.str();
    }
    
    std::function<double(double)> TrajectoryGenerator::CalclateTrajectoryFunction(double init_angle, double init_vel, double goal_angle, double goal_vel, double duration) {
        return [=](double t) {
            double a0 = init_angle;
            double a1 = init_vel;
            double a2 = 3.0 / duration / duration * (goal_angle - init_angle) - 2.0 / duration * init_vel - goal_vel / duration;
            double a3 = - 2.0 / duration / duration / duration * (goal_angle - init_angle) + 1.0 / duration / duration * (goal_vel + init_vel);
            return a0 + a1 * t + a2 * t * t + a3 * t * t * t;
        };
    }
    
    std::function<std::array<double, 6>(double)> TrajectoryGenerator::CalclateTrajectoryFunctions(std::array<double, 6> init_angle, std::array<double, 6> init_vel, std::array<double, 6> goal_angle, std::array<double, 6> goal_vel, double duration) {
        return [=](double t) {
            std::array<double, 6> result;
            for (size_t i = 0; i < 6; ++i) {
                double a0 = init_angle[i];
                double a1 = init_vel[i];
                double a2 = 3.0 / duration / duration * (goal_angle[i] - init_angle[i]) - 2.0 / duration * init_vel[i] - goal_vel[i] / duration;
                double a3 = - 2.0 / duration / duration / duration * (goal_angle[i] - init_angle[i]) + 1.0 / duration / duration * (goal_vel[i] + init_vel[i]);
                result[i] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
            }
            return result;
        };
    }
}
