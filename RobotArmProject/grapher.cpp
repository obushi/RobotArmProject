//
//  gnuplot.cpp
//  RobotArmProject
//
//  Created by NoriyasuObushi on 2018/05/24.
//  Copyright © 2018 NoriyasuObushi. All rights reserved.
//

#include "grapher.hpp"

namespace robot_arm_project {
    
    Grapher::Grapher() {
        fp = popen("gnuplot", "w");
        if (fp == nullptr) {
            std::cout << "Pipe error" << std::endl;
        }
    }
    
    Grapher::Grapher(const char * file_name) {
        fp = fopen(file_name, "w");
        if (fp == nullptr) {
            std::cout << "Pipe error" << std::endl;
        }
    }
    
    Grapher::~Grapher() {
        pclose(fp);
    }
    
    void Grapher::Execute(const char * command, ...) {
        char buffer[1024];
        va_list ap;
        va_start(ap, command);
        vsprintf(buffer, command, ap);
        va_end(ap);
        fprintf(fp, "%s\n", buffer);
        Flush();
    }
    
    void Grapher::Plot(const char * function) {
        Execute("plot %s", function);
    }
    
    void Grapher::Flush() {
        fflush(fp);
    }
    
    void Grapher::SetRangeX(const double min, const double max) {
        Execute("set xrange [%f:%f]", min, max);
    }
    
    void Grapher::SetRangeY(const double min, const double max) {
        Execute("set yrange [%f:%f]", min, max);
    }
}
