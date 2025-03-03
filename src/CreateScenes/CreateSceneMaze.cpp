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
#include "Components/Scene2D.hpp"
#include <fstream>

using namespace Antipatrea;

extern "C" void CreateSceneMaze(int argc, char **argv)
{
    Scene2D scene;
    Maze maze;
    
    const double hdim   = 40;
    const double thick  = 1.0;

    scene.GetGrid()->Setup2D(12, 12, -hdim, -hdim, hdim, hdim);
    maze.m_width = thick;
    maze.m_percKeepBlocked = 1.0;
    maze.m_percAddWallWhenEmpty = 0.0;

    maze.Construct(scene);
    
    
    std::ofstream out("data/SceneMazeObstacles.txt", std::ofstream::out);
    scene.PrintObstacles(out);
    out.close();

    
}


    

    
