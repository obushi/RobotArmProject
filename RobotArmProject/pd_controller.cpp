//
//  pd_controller.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/06/03.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "pd_controller.hpp"

namespace robot_arm_project {
    
    void PDController::Compute(double dt, const std::array<double, 6> & observed, const std::array<double, 6> & target) {
        
        for (std::size_t i = 0; i < 6; ++i)
        {
            double diff_observed = (observed.at(i) - prev_observed.at(i)) / dt;
            double diff_target   = (target.at(i)   - prev_target.at(i))   / dt;
            torque.at(i) = (target.at(i) - observed.at(i)) * kp + (diff_target - diff_observed) * kd;
        }
        
        prev_observed = observed;
        prev_target = target;
    }
}

