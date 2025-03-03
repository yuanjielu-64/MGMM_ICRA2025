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
#include "CreateScenes/CreateScene.hpp"
#include "Components/Scene2D.hpp"
#include <fstream>

using namespace Antipatrea;

extern "C" void CreateSceneRandom(int argc, char **argv) {
    const int GROUP_OBSTACLES = 0;

    CreateScene cs;
    Scene2D scene;

    const double hdim = 0.5 * cs.DefaultDimScene2D();
    const double thick = cs.DefaultThicknessScene2D();
    const int nrObsts = 60;
    double dsep = 12.0;

    cs.m_shapeMinDims[0] = cs.m_shapeMinDims[1] = 3;
    cs.m_shapeMaxDims[0] = cs.m_shapeMaxDims[1] = 5;

    scene.GetGrid()->Setup2D(1, 1, -hdim, -hdim, hdim, hdim);
    cs.SetGrid(scene.GetGrid());

    cs.GetGroups()->push_back(new CreateScene::Group());
    cs.SetSeparation(GROUP_OBSTACLES, GROUP_OBSTACLES, dsep);

    auto obsts = cs.GetGroups()->operator[](GROUP_OBSTACLES);

    obsts->m_tmesh.AddBoundaries2D(-hdim, -hdim, hdim, hdim, thick);
    // obsts->m_tmesh.AddBox2D(-25, -30, -15, -25);

    while (obsts->m_objects.size() < nrObsts && dsep >= 0.1) {
        if (cs.AddObject(GROUP_OBSTACLES, 1000) == false) {
            dsep *= 0.85;
            cs.SetSeparation(GROUP_OBSTACLES, GROUP_OBSTACLES, dsep);
        }

        Logger::m_out << "...generated " << obsts->m_objects.size() << " polygons (dsep = " << dsep << ")" << std::endl;
    }

    std::ofstream out("data/SceneRandomObstacles.txt", std::ofstream::out);
    PrintPolygons2D(out, obsts->m_objects);
    out.close();


}


    

    
