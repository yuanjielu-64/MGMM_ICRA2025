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
#include "Components/RegionPolygon2D.hpp"
#include "Components/RegionBox3D.hpp"
#include "CreateScenes/Constants.hpp"
#include "CreateScenes/CreateScene.hpp"
#include "Programs/Constants.hpp"
#include "Programs/Setup.hpp"
#include <fstream>

using namespace Antipatrea;

extern "C" int GenerateInstances(int argc, char **argv)
{

    const int GROUP_OBSTACLES = 0;
    const int GROUP_INITIAL = 1;
    const int GROUP_GOALS = 2;

    Setup setup;
    Params params;
    CreateScene cs;

    if (argc < 1)
        return 0;

    params.ReadFromFile(argv[1]);
    params.ProcessArgs(2, argc - 1, argv);

    setup.SetupFromParams(params);

    cs.SetGrid(setup.GetScene()->GetGrid());
    cs.GetGroups()->push_back(new CreateScene::Group());
    cs.GetGroups()->push_back(new CreateScene::Group());
    cs.GetGroups()->push_back(new CreateScene::Group());

    cs.m_shapeProbL = 0.0;
    cs.m_shapeProbU = 0.0;
    cs.m_shapeProbRect = 1.1;
    cs.m_shapeMinDims[0] = 3.0;
    cs.m_shapeMinDims[1] = 3.0;
    cs.m_shapeMaxDims[0] = 3.0 + Constants::EPSILON;
    cs.m_shapeMaxDims[1] = 3.0 + Constants::EPSILON;
    cs.m_shapeMinAngle = 0.0;
    cs.m_shapeMaxAngle = 0.0;

    auto gobs = cs.GetGroups()->operator[](GROUP_OBSTACLES);
    auto ginit = cs.GetGroups()->operator[](GROUP_INITIAL);
    auto ggoals = cs.GetGroups()->operator[](GROUP_GOALS);
    int nrGoals = 0;
    double cW = 1.25;
    double cL = 3.0;
    double init[6] = {-0.5 * cL, -0.5 * cW, 0.5 * cL, 0.5 * cW};

    gobs->m_tmesh.AddTriMesh(*(setup.GetScene()->GetObstaclesCollisionMesh()));

    auto data = params.GetData(Constants::KW_GenerateInstances);
    if (data && data->m_params)
    {
        cs.SetupFromParams(*(data->m_params));
        cs.SetSeparation(GROUP_OBSTACLES, GROUP_OBSTACLES,
                         data->m_params->GetValueAsDouble(Constants::KW_SeparationObstacleObstacle, Constants::SEPARATION_OBSTACLE_OBSTACLE));
        cs.SetSeparation(GROUP_OBSTACLES, GROUP_INITIAL,
                         data->m_params->GetValueAsDouble(Constants::KW_SeparationObstacleInitial, Constants::SEPARATION_OBSTACLE_INITIAL));
        cs.SetSeparation(GROUP_OBSTACLES, GROUP_GOALS, data->m_params->GetValueAsDouble(Constants::KW_SeparationObstacleGoal, Constants::SEPARATION_OBSTACLE_GOAL));
        cs.SetSeparation(GROUP_INITIAL, GROUP_INITIAL,
                         data->m_params->GetValueAsDouble(Constants::KW_SeparationInitialInitial, Constants::SEPARATION_INITIAL_INITIAL));
        cs.SetSeparation(GROUP_INITIAL, GROUP_GOALS, data->m_params->GetValueAsDouble(Constants::KW_SeparationInitialGoal, Constants::SEPARATION_INITIAL_GOAL));
        cs.SetSeparation(GROUP_GOALS, GROUP_GOALS, data->m_params->GetValueAsDouble(Constants::KW_SeparationGoalGoal, Constants::SEPARATION_GOAL_GOAL));

        nrGoals = data->m_params->GetValueAsInt(Constants::KW_NrGoals, nrGoals);
        data->m_params->GetValuesAsDoubles(Constants::KW_Initial, init, 2 * cs.GetGrid()->GetNrDims());

    }

    double rminInitGoal = params.GetValueAsDouble("GenerateInstancesInitialGoalMinRadius",  -1);
    double rmaxInitGoal = params.GetValueAsDouble("GenerateInstancesInitialGoalMaxRadius",  -1);

    double x1 = params.GetValueAsDouble("x1",  -1);
    double y1 = params.GetValueAsDouble("y1",  -1);
    double x2 = params.GetValueAsDouble("x2",  -1);
    double y2 = params.GetValueAsDouble("y2",  -1);

    nrGoals = params.GetValueAsInt("GenerateInstancesNrGoals", nrGoals);

    int count = 0;
    double T[3];
    TriMeshDefault tmesh;

    do{
        T[0] = x1 + RandomUniformReal(-2, 2);
        T[1] = y1 + RandomUniformReal(-2, 2);
        tmesh.Clear();
        tmesh.AddBox2D(init[0] + T[0], init[1] + T[1], init[2] + T[0], init[3] + T[1]);
        count ++;

        if (count == 12){
            //Logger::m_out << "we fail!!!!!!!!! 1" << std::endl;
            const char *fname = params.GetValue("GenerateInstancesWriteToFile");
            if(fname == NULL)
                fname = data->m_params->GetValue(Constants::KW_WriteToFile);
            std::ofstream out(fname);
            out << "-1" << std::endl;
            return -1;
        }

    }while (cs.IsAcceptable(GROUP_INITIAL, tmesh) == false);

    ginit->m_tmesh.AddTriMesh(tmesh);

    count = 0;
    while (ggoals->m_objects.size() < nrGoals && ggoals->m_boxes3D.size() < 6 * nrGoals)
    {

        while (cs.AddObject(GROUP_GOALS, 50, rminInitGoal,  rmaxInitGoal, T, x2, y2) == false){

            cs.SetSeparation(GROUP_GOALS, GROUP_OBSTACLES, cs.GetSeparation(GROUP_GOALS, GROUP_OBSTACLES) * 0.90);
            cs.SetSeparation(GROUP_GOALS, GROUP_INITIAL, cs.GetSeparation(GROUP_GOALS, GROUP_INITIAL) * 0.90);
            cs.SetSeparation(GROUP_GOALS, GROUP_GOALS, cs.GetSeparation(GROUP_GOALS, GROUP_GOALS) * 0.90);
            //Logger::m_out << "...generating initial placement: trial = " << count << " sep = " << cs.GetSeparation(GROUP_INITIAL, GROUP_OBSTACLES) << std::endl;
            count ++;
            //Logger::m_out << "count" << count << std::endl;

            if (count == 10){
                //Logger::m_out << "we fail!!!!!!!!! 2" << std::endl;
                const char *fname = params.GetValue("GenerateInstancesWriteToFile");
                if(fname == NULL)
                    fname = data->m_params->GetValue(Constants::KW_WriteToFile);
                std::ofstream out(fname);
                out << "-2" << std::endl;
                return -1;
            }
        }

        //Logger::m_out << "...generated " << ggoals->m_objects.size() << " polygons and " << (ggoals->m_boxes3D.size() / 6) << " 3D boxes goals " << std::endl;
    }

    //Logger::m_out << "...setting initial state" << std::endl;
    std::vector<double> s;
    s.resize(setup.GetSimulator()->GetStateAllocator()->GetDim());
    setup.GetSimulator()->GetState(&s[0]);
    setup.GetSimulator()->SetPositionState(T, &s[0]);
    setup.GetSimulator()->SetState(&s[0]);
    //Logger::m_out << "...initial state: ";
    setup.GetSimulator()->GetStateAllocator()->Print(Logger::m_out, &s[0]);

    //Logger::m_out << "...adding goals to problem" << std::endl;
    DeleteItems<RegionGeometric *>(*(setup.GetProblem()->GetGoals()));
    setup.GetProblem()->GetGoals()->clear();
    setup.GetProblem()->GetTimeBounds()->clear();
    if(cs.GetGrid()->GetNrDims() == 2)
        for (int i = 0; i < (int)ggoals->m_objects.size(); ++i)
        {
            auto poly = ggoals->m_objects[i];
            auto r = new RegionPolygon2D();
            r->SetFlags(RemoveAndAddFlags(r->GetFlags(), Region::STATUS_REGULAR, Region::STATUS_GOAL));
            r->GetPolygon2D()->AddVertices(poly->GetNrVertices(), &(poly->GetVertices()->operator[](0)));
            setup.GetProblem()->GetGoals()->push_back(r);
        }
    else
        for (int i = 0; i < (int)ggoals->m_boxes3D.size();  i  += 6)
        {
            auto box = &(ggoals->m_boxes3D[i]);
            auto r = new RegionBox3D();
            r->SetFlags(RemoveAndAddFlags(r->GetFlags(), Region::STATUS_REGULAR, Region::STATUS_GOAL));
            r->SetMinMax(box);
            setup.GetProblem()->GetGoals()->push_back(r);
        }

    std::vector<double> aux;
    aux.resize(setup.GetSimulator()->GetCfgAllocator()->GetDim());
    setup.GetSimulator()->GetCfgState(&s[0], &aux[0]);
    auto rid = setup.GetDecomposition()->LocateRegion(&aux[0]);
    //Logger::m_out << "...initial region: " << rid << std::endl;

    //Logger::m_out << "...computing time bounds" << std::endl;
    std::vector<int> perm;
    std::vector<double> times;
    double t;

    perm.resize(setup.GetProblem()->GetGoals()->size());
    times.resize(perm.size());

    const char *fname = params.GetValue("GenerateInstancesWriteToFile");
    if(fname == NULL)
        fname = data->m_params->GetValue(Constants::KW_WriteToFile);
    std::ofstream out(fname);

    //Logger::m_out << "writing to file <" << fname << ">" << std::endl;

    out << Constants::KW_Problem << std::endl
        << "{" << std::endl;
    if(cs.GetGrid()->GetNrDims() == 2)
        out << Constants::KW_CurrentState << " [ " << T[0] << " " << T[1] << " ] " << std::endl;
    else
        out << Constants::KW_CurrentState << " [ " << T[0] << " " << T[1] << " " << T[2] << " ] " << std::endl;
    out << Constants::KW_NrGoals << " " << times.size() << std::endl;
    for (int i = 0; i < (int)times.size(); ++i)
    {
        out << Constants::KW_Goal << i << " { ";
        if(cs.GetGrid()->GetNrDims() == 2)
        {
            out  << Constants::KW_Polygon2D << " [ ";
            auto vertices = dynamic_cast<RegionPolygon2D *>(setup.GetProblem()->GetGoals()->operator[](i))->GetPolygon2D()->GetVertices();
            for (auto &v : *vertices)
                out << v << " ";
        }
        else
        {
            out  << Constants::KW_Box3D << " [ ";
            auto box = dynamic_cast<RegionBox3D *>(setup.GetProblem()->GetGoals()->operator[](i))->GetMinMax();
            for (int j = 0; j < 6; ++j)
                out << box[j] << " ";

        }

        out << " ] " << Constants::KW_TimeBounds << " [  0 1000000  ] } " << std::endl;
    }

    out << "}" << std::endl;

    return 0;
}

