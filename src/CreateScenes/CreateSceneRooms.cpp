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
#include "Components/Scene3D.hpp"
#include "Utils/TriMeshStreamer.hpp"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace Antipatrea;


extern "C" void CreateSceneRooms(int argc, char **argv)
{
    Scene3D          scene;
    std::vector<int> blocks; 
    Fence            fence;
  
    const double hdim   = 100;
    const double zmax   = 26;
    const double thick  = 0.5;
    
    const double P1[] = {-hdim, -50.0};
    const double P2[] = {-40, -70.0};
    const double P3[] = {0, -30.0};
    const double P4[] = {40, P3[1]};
    const double P5[] = {hdim, P4[1] - 20};
    const double P6[] = {P2[0] - 5, -hdim};
    const double P7[] = {P3[0] + 15, -70};
    const double P8[] = {P7[0] - 15, -hdim};
    const double P9[] = {P7[0] + 60, P7[1]};
    const double P10[] = {P9[0] - 15, -hdim};
    const double P11[] = {P2[0] - 20, P2[1] + 70};
    const double P12[] = {-hdim, P11[1]};
    const double P13[] = {P3[0] - 25, P3[1] + 50};
    const double P14[] = {P13[0] + 80 , P13[1]};
    const double P15[] = {hdim , P14[1] - 10};
    const double P16[] = {P11[0] + 10 , P11[1] + 60};
    const double P17[] = {-hdim, P16[1] - 10};
    const double P18[] = {P16[0] + 50 , P16[1] + 10};
    const double P19[] = {P18[0] - 30 , hdim};
    const double P20[] = {P14[0] - 10 , hdim};

    GMaterial *gmatFloor = new GMaterial();
    GMaterial *gmatWalls = new GMaterial();
    

    auto tmeshDraw = scene.GetObstaclesDrawMesh();

    gmatFloor->SetYellowPlastic();
    gmatWalls->SetTurquoise();
    tmeshDraw->AddMaterial(gmatWalls);
    tmeshDraw->AddMaterial(gmatFloor);
    
    tmeshDraw->SetCurrentMaterial(0);
    

    //P1P2			      
    blocks.resize(3);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 2, 4);
    fence.Construct(*tmeshDraw, 3, 2, P1[0], P1[1], P2[0], P2[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);


    //P2P3			      
    blocks.resize(6);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 4, 6, 7, 8);
    fence.Construct(*tmeshDraw, 3, 3, P2[0], P2[1], P3[0], P3[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

    
    //P3P4			      
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 4, 5);
    fence.Construct(*tmeshDraw, 3, 2, P3[0], P3[1], P4[0], P4[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

        //P4P5			      
    blocks.resize(8);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 2, 3, 5, 7, 9, 10, 11);
    fence.Construct(*tmeshDraw, 4, 3, P4[0], P4[1], P5[0], P5[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

           //P2P6			      
    blocks.resize(7);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 2, 3, 4, 5, 6, 7);
    fence.Construct(*tmeshDraw, 3, 3, P2[0], P2[1], P6[0], P6[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);


           //P3P7			      
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 3, 4);
    fence.Construct(*tmeshDraw, 2, 3, P3[0], P3[1], P7[0], P7[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

               //P7P8 
    blocks.resize(7);
    AssignItems<int>(&blocks[0], blocks.size(), 1, 2, 3, 4, 6, 7, 8);
    fence.Construct(*tmeshDraw, 3, 3, P7[0], P7[1], P8[0], P8[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);


               //P7P9
    blocks.resize(9);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 2, 4, 6, 7, 9, 10, 11);
    fence.Construct(*tmeshDraw, 4, 3, P7[0], P7[1], P9[0], P9[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

                   //P9P5
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 2, 5 );
    fence.Construct(*tmeshDraw, 2, 3, P9[0], P9[1], P5[0], P5[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

    //P9P10
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 4, 5, 8);
    fence.Construct(*tmeshDraw, 3, 3, P9[0], P9[1], P10[0], P10[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

    
    //P2P11
    blocks.resize(6);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 2, 3, 5, 6, 7);
    fence.Construct(*tmeshDraw, 3, 3, P2[0], P2[1], P11[0], P11[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

        //P11P12
    blocks.resize(9);
    AssignItems<int>(&blocks[0], blocks.size(), 1, 2, 3, 4, 5, 7, 9, 10, 11);
    fence.Construct(*tmeshDraw, 4, 3, P11[0], P12[1], P12[0], P12[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);


        //P3P13
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 1, 2, 3, 4);
    fence.Construct(*tmeshDraw, 2, 3, P3[0], P3[1], P13[0], P13[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

        //P11P13
    blocks.resize(5);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 2, 3, 5);
    fence.Construct(*tmeshDraw, 3, 2, P11[0], P11[1], P13[0], P13[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

            //P13P14
    blocks.resize(6);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 3, 4, 6, 7);
    fence.Construct(*tmeshDraw, 4, 2, P13[0], P13[1], P14[0], P14[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

                //P4P14
    blocks.resize(6);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 2, 3, 4, 6, 8);
    fence.Construct(*tmeshDraw, 3, 3, P4[0], P4[1], P14[0], P14[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

                   //P14P15
    blocks.resize(2);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 3);
    fence.Construct(*tmeshDraw, 2, 2, P14[0], P14[1], P15[0], P15[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

                //P11P16
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 2, 3, 4);
    fence.Construct(*tmeshDraw, 3, 2, P11[0], P11[1], P16[0], P16[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

                    //P16P17
    blocks.resize(6);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 4, 5, 6, 7);
    fence.Construct(*tmeshDraw, 3, 3, P16[0], P16[1], P17[0], P17[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

                        //P13P18
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 2, 3, 4);
    fence.Construct(*tmeshDraw, 3, 2, P13[0], P13[1], P18[0], P18[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

                      //P18P19
    blocks.resize(2);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 3);
    fence.Construct(*tmeshDraw, 2, 2, P18[0], P18[1], P19[0], P19[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

    //P14P20
    blocks.resize(4);
    AssignItems<int>(&blocks[0], blocks.size(), 0, 1, 3, 5);
    fence.Construct(*tmeshDraw, 3, 2, P14[0], P14[1], P20[0], P20[1], thick, 0, zmax, blocks);
    PrintItems<int>(std::cout, blocks);

    tmeshDraw->SetCurrentMaterial(1);    
    tmeshDraw->AddBox2D(-hdim, -hdim, hdim, hdim);

    
    const int n = scene.GetObstaclesCollisionMesh()->GetNrVertices();
    scene.GetObstaclesCollisionMesh()->AddBox2D(-hdim, -hdim, hdim, hdim);
    scene.GetObstaclesCollisionMesh()->ApplyTrans(n, scene.GetObstaclesCollisionMesh()->GetNrVertices() - 1, 0.0, 0.0, zmax);
    scene.GetObstaclesCollisionMesh()->AddTriMesh(*tmeshDraw);
    scene.GetObstaclesCollisionMesh()->AddBoundaries(-hdim, -hdim, 0, hdim, hdim, zmax, thick);

    tmeshDraw->SetCurrentMaterial(0);
    tmeshDraw->AddBoundaries(-hdim, -hdim, 0, hdim, hdim, 4.0, thick);
    
    
    std::ofstream out("data/SceneRoomsDraw.tmesh", std::ofstream::out);    
    StandardTriMeshWriter(out, *tmeshDraw);
    out.close();

    
    out.open("data/SceneRoomsCollision.tmesh", std::ofstream::out);    
    StandardTriMeshWriter(out, *(scene.GetObstaclesCollisionMesh()));
    out.close();
    
    
}


    

    
