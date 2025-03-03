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
#include "CreateScenes/Maze.hpp"
#include "Utils/DisjointSet.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Misc.hpp"

namespace Antipatrea
{
    void Maze::GenerateKruskal(const int dimsx, const int dimsy)
    {
	m_blocked.clear();
	m_empty.clear();
	
	DisjointSet                     dset;
	std::vector<DisjointSet::Elem*> cells;
	std::vector<Border>             walls;
	Border                          wall;
	int                             nrBlocked = 0;
	
	for(int x = 0; x < dimsx; ++x)
	    for(int y = 0; y < dimsy; ++y)
	    {
		if(x > 0)
		{
		    wall.m_cids[0] = y * dimsx + x;
		    wall.m_cids[1] = wall.m_cids[0] - 1;
		    walls.push_back(wall);
		}
		if(y > 0)
		{
		    wall.m_cids[0] = y * dimsx + x;
		    wall.m_cids[1] = wall.m_cids[0] - dimsx;
		    walls.push_back(wall);
		}
		cells.push_back(dset.Make());
	    }

	PermuteItems<Border>(walls, walls.size());
	
	for(int i = 0; i < (int) (walls.size()); ++i)
	{
	    wall = walls[i];
	    if(dset.Same(cells[wall.m_cids[0]], cells[wall.m_cids[1]]) == false)
	    {
		m_empty.push_back(wall);
		dset.Join(cells[wall.m_cids[0]], cells[wall.m_cids[1]]);
	    }
	    else
		m_blocked.push_back(wall);
	}

	DeleteItems<DisjointSet::Elem*>(cells);
    }

    void Maze::KeepBlocked(void)
    {
	const int nuse = m_blocked.size() * m_percKeepBlocked;
	for(int i = m_blocked.size() - 1; i >= nuse; --i)
	{
	    m_empty.push_back(m_blocked.back());
	    m_blocked.pop_back();
	}
    }

    void Maze::Construct(Scene2D & scene)
    {
	double min[2];
	double max[2];
	double c[2];	
	double TR[Algebra2D::TransRot_NR_ENTRIES];
	Polygon2D *poly;
	int        nuse;
	std::vector<double> vertices;
	
	
	GenerateKruskal(scene.GetGrid()->GetDims()[0], scene.GetGrid()->GetDims()[1]);
	KeepBlocked();
	
//add blocked walls	
	for(int i = 0; i < m_blocked.size(); ++i)
	{
	    GetWall(*(scene.GetGrid()), m_blocked[i], 1 - Constants::EPSILON, min, max);//1.00 - Constants::EPSILON, min, max);
		
	    poly = new Polygon2D();
	    vertices.resize(8);
	    BoxAsPolygon2D(min, max, &(vertices[0]));
	    poly->AddVertices(vertices.size() / 2, &vertices[0]);
	    scene.AddObstacle(poly);
	}
 
//add random wall when empty
	nuse = m_empty.size() * m_percAddWallWhenEmpty;
	for(int i = 0; i < nuse; ++i)
	{
	    GetWall(*(scene.GetGrid()), m_empty[i], RandomUniformReal(0.4, 0.7), min, max);
	    c[0] = 0.5 * (min[0] + max[0]);
	    c[1] = 0.5 * (min[1] + max[1]);
	    Algebra2D::RotateAroundPointAsTransRot(RandomUniformReal(-M_PI, M_PI), c, TR);

	    poly = new Polygon2D();
	    vertices.resize(8);
	    BoxAsPolygon2D(min, max, &(vertices[0]));
	    ApplyTransRotToPolygon2D(TR, 4, &(vertices[0]), &(vertices[0]));
	    poly->AddVertices(vertices.size() / 2, &vertices[0]);
	    scene.AddObstacle(poly);
	}
    }

    void Maze::GetWall(const Grid  &  grid, 
		       const Border         wall, 
		       const double         percKeepLength,
		       double               min[2], 
		       double               max[2]) const
    {
	const double *gmin     = grid.GetMin();
	const int    *dims     = grid.GetDims();
	const double *units    = grid.GetUnits();
	const int     coords[] = {wall.m_cids[0] % dims[0], wall.m_cids[0] / dims[0]};
	const int     which    = (wall.m_cids[0] == (wall.m_cids[1] + 1)) ? 0 : 1;
	
	min[which]     = gmin[which]     + coords[which] * units[which] - 0.5 * m_width;
	min[1 - which] = gmin[1 - which] + coords[1 - which] * units[1 - which] + 0.5 * (1 - percKeepLength) * units[1 - which];
	max[which]     = min[which] + m_width;
	max[1 - which] = min[1 - which] + percKeepLength * units[1 - which];
/*
	if(percKeepLength >= 0.6)
	{
	    if(which == 1) // horizontal
	    {
		min[0] = min[0] + 0.5 * m_width;
		max[0] = max[0] - 0.5 * m_width;
	    }
	    
	}
*/	
    }

    
}




