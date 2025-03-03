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
#include "Programs/Setup.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Components/TourGeneratorExact.hpp"
#include <fstream>

using namespace Antipatrea;

extern "C" int RunDistance(int argc, char **argv)
{
    Setup setup;
    Params params;
   
    params.ReadFromFile(argv[1]);
    params.ProcessArgs(2, argc - 1, argv);

    setup.SetupFromParams(params);
    setup.RunConstructDecomposition(params);
    setup.GetMP()->Start();

    auto fnameOut = params.GetValue("OutputFile", "data/out.txt");
    std::ofstream out(fnameOut);
    
    
    auto decomp = setup.GetDecomposition();
    auto goals = setup.GetProblem()->GetGoals();
    std::vector<int> rids;

    rids.push_back(0);
    for(int i = 0;  i < goals->size(); ++i)
   	rids.push_back((*goals)[i]->GetKey());
    //for(int i = 0; i < rids.size(); ++i)
//	Logger::m_out << "rids[" << i << "] = " << rids[i] << std::endl;

    
    double p[3];    
    auto ss = setup.GetMP()->GetState(0);
    out << ss[0] << " " << ss[1] << " ";

    for(int i = 0; i < goals->size(); ++i)
    {
	(*goals)[i]->GetRepresentativePoint(p);
	out << p[0] << " " << p[1]  << " ";
    }
    for(int i = 0; i < rids.size(); ++i)
    {
	for(int j = 0; j < goals->size(); ++j)
	{
	    auto goal = (*goals)[j];
	    auto data = dynamic_cast<Region*>(decomp->GetGraph()->GetVertex(rids[i]))->GetPathDataToGoals()->operator[](j);

	    if(rids[i] < goal->GetKey())
		out << data->m_cost << " ";	    
	}	
    }
    out << std::endl;
    
    
    out.close();
    
    return 0;
    
}

