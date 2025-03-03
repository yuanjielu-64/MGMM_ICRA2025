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
#include "Programs/Constants.hpp"
#include "Programs/GManagerDecomposition.hpp"
#include "Programs/GManagerGoals.hpp"
#include "Programs/GManagerMP.hpp"
#include "Programs/GManagerSimulator.hpp"
#include "Utils/GManager.hpp"

using namespace Antipatrea;

extern "C" int GRunMP(int argc, char **argv) {
    GManager gManager;
    GManagerMP gMP;
    GManagerDecomposition gDecomposition;
    GManagerGoals gGoals;
    GManagerSimulator gSimulator;
    Setup setup;
    Params params;

    if (argc < 1)
        return 0;

    params.ReadFromFile(argv[1]);
    params.ProcessArgs(2, argc - 1, argv);

    setup.SetupFromParams(params);
    setup.RunConstructDecomposition(params);

    gDecomposition.SetSetup(&setup);
    gDecomposition.SetManager(&gManager);
    gManager.GetComponents()->push_back(&gDecomposition);

    gGoals.SetSetup(&setup);
    gGoals.SetManager(&gManager);
    gManager.GetComponents()->push_back(&gGoals);

    gSimulator.SetSetup(&setup);
    gSimulator.SetManager(&gManager);
    gManager.GetComponents()->push_back(&gSimulator);

    gMP.SetSetup(&setup);
    gMP.SetManager(&gManager);
    gManager.GetComponents()->push_back(&gMP);

    auto data = params.GetData(Antipatrea::Constants::KW_Graphics);
    if (data && data->m_params) {
        GDrawSetupFromParams(*(data->m_params));
        gManager.SetupFromParams(*(data->m_params));
        gMP.SetupFromParams(*(data->m_params));
        gDecomposition.SetupFromParams(*(data->m_params));
        gGoals.SetupFromParams(*(data->m_params));
        gSimulator.SetupFromParams(*(data->m_params));
    }


    gManager.MainLoop("GRunMP");

    return 0;
}
