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
#include "CreateScenes/Fence.hpp"

namespace Antipatrea
{
    
    void Fence::Construct(TriMesh &tmesh,
			  const int    dimsBase,
			  const int    dimsZ,
			  const double x1,
			  const double y1,
			  const double x2,
			  const double y2,
			  const double thick,
			  const double zmin,
			  const double zmax,
			  const std::vector<int> & blocks)
    {

	const double norm = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	const double vx   = (x2 - x1) / norm;
	const double vy   = (y2 - y1) / norm;
	const double nx   = -vy;;
	const double ny   =  vx;
	double       sx1;
	double       sx2;
	double       sy1;
	double       sy2;
	double       gapB;
	double       gapZ;
	Polygon2D    poly;
	std::vector<double> vertices;
	
	
	vertices.resize(8);
	
	//parallel to the base line: so, only height changes
	vertices[0] = x1 - nx * 0.5 * thick;  vertices[1] = y1 - ny * 0.5 * thick;
	vertices[2] = x2 - nx * 0.5 * thick;  vertices[3] = y2 - ny * 0.5 * thick;
	vertices[4] = x2 + nx * 0.5 * thick;  vertices[5] = y2 + ny * 0.5 * thick;
	vertices[6] = x1 + nx * 0.5 * thick;  vertices[7] = y1 + ny * 0.5 * thick;
	poly.Clear();
	poly.AddVertices(vertices.size() / 2, &vertices[0]);
	
	
	gapZ = (zmax - zmin - (dimsZ + 1) * thick) / dimsZ;
	for(int i = 0; i <= dimsZ; ++i)
	    tmesh.AddExtrudedPolygon(poly, zmin + i * (gapZ + thick), zmin + i * (gapZ + thick) + thick);
	
	//vertical to the xy plane
	gapB = (norm - (dimsBase + 1) * thick) / dimsBase;
	for(int i = 0; i <= dimsBase; ++i)
	{
	    sx1 = x1 + vx * i * (gapB + thick);
	    sy1 = y1 + vy * i * (gapB + thick);
	    sx2 = sx1 + vx * thick;
	    sy2 = sy1 + vy * thick;
	    vertices[0] = sx1 - nx * 0.5 * thick;  vertices[1] = sy1 - ny * 0.5 * thick;
	    vertices[2] = sx2 - nx * 0.5 * thick;  vertices[3] = sy2 - ny * 0.5 * thick;
	    vertices[4] = sx2 + nx * 0.5 * thick;  vertices[5] = sy2 + ny * 0.5 * thick;
	    vertices[6] = sx1 + nx * 0.5 * thick;  vertices[7] = sy1 + ny * 0.5 * thick;
	    poly.Clear();
	    poly.AddVertices(vertices.size() / 2, &vertices[0]);
	    tmesh.AddExtrudedPolygon(poly, zmin + thick, zmax - thick);
	}
	
	for(int i = blocks.size() - 1; i >= 0; --i)
	{
	    const int bi = (blocks[i]) % dimsBase;
	    const int zi = (blocks[i]) / dimsBase;
	    
	    sx1 = x1 + vx * bi * (gapB + thick) + vx * thick;
	    sy1 = y1 + vy * bi * (gapB + thick) + vy * thick;
	    sx2 = sx1 + vx * gapB;
	    sy2 = sy1 + vy * gapB;
	    vertices[0] = sx1 - nx * 0.5 * thick;  vertices[1] = sy1 - ny * 0.5 * thick;
	    vertices[2] = sx2 - nx * 0.5 * thick;  vertices[3] = sy2 - ny * 0.5 * thick;
	    vertices[4] = sx2 + nx * 0.5 * thick;  vertices[5] = sy2 + ny * 0.5 * thick;
	    vertices[6] = sx1 + nx * 0.5 * thick;  vertices[7] = sy1 + ny * 0.5 * thick;
	    poly.Clear();
	    poly.AddVertices(vertices.size() / 2, &vertices[0]);
	    tmesh.AddExtrudedPolygon(poly, zmin + zi * (gapZ + thick) + thick, zmin + zi * (gapZ + thick) + thick + gapZ);
	}
    }
    
}
