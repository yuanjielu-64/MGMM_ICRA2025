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

#ifndef Antipatrea_Fence_HPP_
#define Antipatrea_Fence_HPP_

#include "Components/Scene.hpp"

namespace Antipatrea
{
    class Fence
    {
    public:
	Fence(void)
	{
	}

	virtual ~Fence(void)
	{
	}
	
	
	virtual void Construct(TriMesh &tmesh,
			       const int    dimsBase,
			       const int    dimsZ,
			       const double x1,
			       const double y1,
			       const double x2,
			       const double y2,
			       const double thick,
			       const double zmin,
			       const double zmax,
			       const std::vector<int> & blocks);
    };
    	
}


#endif
