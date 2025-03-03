/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */
#include <fstream>


extern "C" int ProcessResults(int argc, char **argv)
{
    auto fnameAll = argv[1];
    auto fnameNormal= argv[2];
    auto fnameReverse = argv[3];

    
    int solved;
    double pos[6];
    double t;
    int order[2];
    double tsumNormal = 0;
    int  nNormal= 0;
    double tsumReverse = 0;
    int nReverse = 0;
    

    std::ifstream in(fnameNormal);
    while(in >> solved >> pos[0] >> pos[1] >> pos[2] >> pos[3] >> pos[4] >> pos[5]>> t >> order[0] >> order[1])
    {
	tsumNormal += t;	
	++nNormal;	
    }
    in.close();

    in.open(fnameReverse);
    while(in >> solved >> pos[0] >> pos[1] >> pos[2] >> pos[3] >> pos[4] >> pos[5]>> t >> order[0] >> order[1])
    {
	tsumReverse += t;	
	++nReverse;	
    }
    in.close();

    
    std::ofstream out(fnameAll, std::ios::app);
    out << pos[0] << " " << pos[1] << " " << pos[2] << " " << pos[3] << " " << pos[4] << " " << pos[5] << " " << (tsumNormal / nNormal) << " " << (tsumReverse/nReverse) << std::endl;
    
    out.close();
    
    return 0;
    
}

