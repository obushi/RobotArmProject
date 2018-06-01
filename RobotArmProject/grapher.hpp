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
        
        template <class T, int N>
        void Plot(T (&cont)[N])
        {
            int x = 0;
            Execute("plot '-' w lp");
            while (x < N) {
                Command("%f", cont[x]);
                x++;
            }
            Execute("e");
        }
        
        template <class T, int N, int M>
        void Plot(T (&contX)[N], T (&contY)[M])
        {
            int x = 0;
            int y = 0;
            Execute("plot '-' w lp");
            while (x < N && y < M) {
                Execute("%f %f", contX[x], contY[y]);
                x++; y++;
            }
            Execute("e");
        }
        
        void SetRangeX(const double min, const double max);
        void SetRangeY(const double min, const double max);
    };
}
