//
//  gnuplot.hpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#pragma once

#include <fstream>
#include <iostream>

namespace robot_arm_project {
    
    class Grapher {
        FILE* fp;
        void Flush();
        
    public:
        Grapher();
        Grapher(const char * file_name);
        ~Grapher();
        void Execute(const char * command, ...);
        void Plot(const char * function);
        void SetRangeX(const double min, const double max);
    };
}
