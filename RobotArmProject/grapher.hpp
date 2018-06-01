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
        
        template <class T>
        void Plot(T const &series)
        {
            int x = 0;
            Execute("plot '-' w lp");
            while (x < series.size()) {
                Execute("%f", series.at(x));
                x++;
            }
            Execute("e");
        }
        
        template <class T>
        void Plot(T const & seriesX, T const & seriesY)
        {
            int x = 0;
            int y = 0;
            Execute("plot '-' w lp");
            while (x < seriesX.size() && y < seriesY.size()) {
                Execute("%f %f", seriesX.at(x), seriesY.at(y));
                x++; y++;
            }
            Execute("e");
        }
        
        template <class T>
        void Plot(T const & seriesX, T const & seriesY, T const & seriesZ)
        {
            int x = 0;
            int y = 0;
            int z = 0;
            Execute("splot '-' w lp");
            while (x < seriesX.size() && y < seriesY.size() && z < seriesZ.size()) {
                Execute("%f %f %f", seriesX.at(x), seriesY.at(y), seriesY.at(z));
                x++; y++; z++;
            }
            Execute("e");
        }
        
        void SetRangeX(const double min, const double max);
        void SetRangeY(const double min, const double max);
        void SetRangeZ(const double min, const double max);
    };
}
