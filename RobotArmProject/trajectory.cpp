//
//  trajectory_generator.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "trajectory.hpp"

namespace robot_arm_project {
    
    Trajectory::Trajectory(double init_angle, double init_velocity, double goal_angle, double goal_velocity, double duration) : init_angle(init_angle), init_velocity(init_velocity), goal_angle(goal_angle), goal_velocity(goal_velocity), duration(duration) {
    }
    
    std::string Trajectory::ToExpression() {
        double a0 = init_angle;
        double a1 = init_velocity;
        double a2 = 3.0 / duration / duration * (goal_angle - init_angle) - 2.0 / duration * init_velocity - goal_velocity / duration;
        double a3 = - 2.0 / duration / duration / duration * (goal_angle - init_angle) + 1.0 / duration / duration * (goal_velocity + init_velocity);
        std::stringstream str;
        str << a3 << "*x**3+" << a2 << "*x**2+" << a1 << "*x+" << a0;
        return str.str();
    }
    
    std::function<double(double)> Trajectory::ToFunction() {
        return [=](double t) {
            double a0 = init_angle;
            double a1 = init_velocity;
            double a2 = 3.0 / duration / duration * (goal_angle - init_angle) - 2.0 / duration * init_velocity - goal_velocity / duration;
            double a3 = - 2.0 / duration / duration / duration * (goal_angle - init_angle) + 1.0 / duration / duration * (goal_velocity + init_velocity);
            return a0 + a1 * t + a2 * t * t + a3 * t * t * t;
        };
    }
    
//    std::function<std::array<double, 6>(double)> Trajectory::GetFunctions(std::array<double, 6> init_angle, std::array<double, 6> init_vel, std::array<double, 6> goal_angle, std::array<double, 6> goal_vel, double duration) {
//        return [=](double t) {
//            std::array<double, 6> result;
//            for (size_t i = 0; i < 6; ++i) {
//                double a0 = init_angle[i];
//                double a1 = init_vel[i];
//                double a2 = 3.0 / duration / duration * (goal_angle[i] - init_angle[i]) - 2.0 / duration * init_vel[i] - goal_vel[i] / duration;
//                double a3 = - 2.0 / duration / duration / duration * (goal_angle[i] - init_angle[i]) + 1.0 / duration / duration * (goal_vel[i] + init_vel[i]);
//                result[i] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
//            }
//            return result;
//        };
//    }
}
