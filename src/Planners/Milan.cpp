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
#include "Planners/Milan.hpp"
#include "Utils/Timer.hpp"

namespace Antipatrea
{

    bool Milan::Solve(const int nrIters, const double tmax, bool & canBeSolved)
{
    Timer::Clock clk;
    std::vector<double> pos;

    Timer::Start(clk);

    GetGroupSelector()->SetGroups(&m_groups);

    if (m_tree.GetNrVertices() == 0)
    {
        if(Start() == false)
	{
	    canBeSolved = false;
	    return false;
	}
    }
    

    pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());
    for (int i = 0; i < nrIters && Timer::Elapsed(clk) && !IsSolved(); ++i)
    {
        GetSimulator()->SamplePosition(&pos[0]);

        auto group = GetGroupSelector()->SelectGroup();

	Logger::m_out << "Selecting " << GetGroupSelector()->GetGroups()->GetNrGroups()<< " with key" << *(group->GetKey()) << " and weight " << group->GetWeight() << std::endl;
	

        ExtendFrom(SelectVertex(*group, &pos[0]), &pos[0]);
    }

    return false;
}
}
