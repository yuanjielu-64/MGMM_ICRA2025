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

#ifndef Antipatrea__Maze_HPP_
#define Antipatrea__Maze_HPP_

#include "Components/Scene2D.hpp"
#include <vector>

namespace Antipatrea
{
    class Maze
    {
    public:
	struct Border
	{
	    int m_cids[2];
	};
	
	double              m_width;
	double              m_percKeepBlocked;
	double              m_percAddWallWhenEmpty;
	std::vector<Border> m_blocked;
	std::vector<Border> m_empty;

	Maze(void) : m_percKeepBlocked(0.98),
		     m_percAddWallWhenEmpty(0.0),
		     m_width(1.0)
	
	{
	}
	
	virtual ~Maze(void)
	{
	}

	virtual void Construct(Scene2D & scene);
	
    protected:	
	virtual void GenerateKruskal(const int dimsx, const int dimsy);

	virtual void KeepBlocked(void);

	virtual void GetWall(const Grid &  grid,
			     const Border   wall,
			     const double       percKeepLength,
			     double             min[2],
			     double             max[2]) const;
    };
}

#endif



