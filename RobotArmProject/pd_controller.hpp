//
//  pd_controller.hpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/06/03.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#pragma once
#include <array>

namespace robot_arm_project {
    
    class PDController {
        
        static constexpr double kp = 1.0;
        static constexpr double kd = 1.0;
        std::array<double, 6> prev_observed;
        std::array<double, 6> prev_target;
        std::array<double, 6> torque;
        
    public:
        void Compute(double dt, const std::array<double, 6> & observed, const std::array<double, 6> & target);
    };
}
