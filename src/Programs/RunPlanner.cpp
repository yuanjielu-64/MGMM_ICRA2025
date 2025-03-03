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
#include "Programs/Setup.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Components/TourGeneratorExact.hpp"
#include <fstream>
#include <iomanip>

using namespace Antipatrea;

extern "C" int RunPlanner(int argc, char **argv) {
    std::ifstream in("stop.txt");
    int check = 1;

    if (in && (in >> check) && check == 1) {
        Logger::m_out << "...forced stop : remove stop.txt to continue" << std::endl;

        in.close();

        return 0;

    }
    if (in)
        in.close();

    Setup setup;
    Params params;

    /*const char *prefix = argv[1];
    const int qi = argc >= 2 ? atoi(argv[2]) : 0;
    char fnameParams[300];
    char fnameProblem[300];
    const bool orderReverse = argc >= 3 ? strcmp(argv[3], "reverse") == 0 : false;
    */

    //sprintf(fnameParams, "data/ParamsScene%sForCar.txt", prefix);
    //sprintf(fnameProblem, "data/Instances/Scene%sForCarNrGoals2_%d.txt", prefix, qi);


    params.ReadFromFile(argv[1]);
    //params.ReadFromFile(fnameProblem);    
    params.ProcessArgs(2, argc - 1, argv);

    /*if(orderReverse)
	params.SetValue("UseTourGenerator", "TourGeneratorExactReverseOrder");
    else
	params.SetValue("UseTourGenerator", "TourGeneratorExactNormalOrder");
    printf("order = %s\n", params.GetValue("UseTourGenerator"));
    */

    auto tmax = params.GetValueAsDouble("PlannerMaxRuntime", 5.0);
    auto recordCannotBeSolved = params.GetValueAsBool("PlannerRecordCannotBeSolved", true);
    std::string statsFileName(params.GetValue("PlannerStatsFile", "stats.txt"));

    int solved = 0;
    Timer::Clock clk;
    double tcurr = 0;
    int count = 0;
    double tint = 2.0;

    Logger::m_out << "...results will be written to <" << statsFileName << ">" << std::endl;


    setup.SetupFromParams(params);
    setup.RunConstructDecomposition(params);

    Timer::Start(clk);
    bool canBeSolved = true;

    while (canBeSolved && (tcurr = Timer::Elapsed(clk)) < tmax && setup.GetMP()->IsSolved() == false) {
        setup.GetMP()->Solve(1000, 7.0, canBeSolved);
        if (tcurr > count * tint) {
            Logger::m_out << "...not solved yet at time " << Timer::Elapsed(clk) << std::endl;
            ++count;

            Logger::m_out << *(Stats::GetSingleton()) << std::endl;

        }
    }
    tcurr = Timer::Elapsed(clk);
    Stats::GetSingleton()->AddValue("TimeSolve", tcurr);

    std::ofstream out(statsFileName.c_str(), std::ofstream::out | std::ofstream::app);
    Solution sol;
    solved = setup.GetMP()->GetSolution(sol);

    const int markAs = solved ? 1 : (canBeSolved == false ? -1 : 0);

    auto ss = setup.GetMP()->GetState(0);
    auto goals = setup.GetProblem()->GetGoals();
    double p[3];

    double totalTime = 0.0;
    totalTime = Stats::GetSingleton()->GetValue("TimeSolve")
                + Stats::GetSingleton()->GetValue("PRMForPredictPath");

    out << totalTime << " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("TimeSolve") << " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("PRMForPredictPath") << " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("TimeForXXX") << " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("TimeCompleteGroup") << " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("TimeCollision") << " "
        << std::setprecision(5) << Stats::GetSingleton()->GetValue("TimeSimulate") << " "

        << totalTime
            - Stats::GetSingleton()->GetValue("TimeForXXX")
            - Stats::GetSingleton()->GetValue("PRMForPredictPath")
            - Stats::GetSingleton()->GetValue("TimeCompleteGroup")
            - Stats::GetSingleton()->GetValue("TimeCollision")
            - Stats::GetSingleton()->GetValue("TimeSimulate") << " "

        << sol.GetCost() << " "
        << sol.GetEndTime() << " "

        << ss[0] << " "
        << ss[1] << " ";

    for (auto & goal : *goals) {
        goal->GetRepresentativePoint(p);
        out << p[0] << " " << p[1] << " ";
    }
    out << std::endl;

    out.close();

    Logger::m_out << "[solved            = " << markAs << "] " << std::endl
                  << "[TimeSolve         = " << Stats::GetSingleton()->GetValue("TimeSolve") << "] " << std::endl
                  << "[SolDistance       = " << sol.GetCost() << "] " << std::endl
                  << "[SolEndTime        = " << sol.GetEndTime() << "]" << std::endl;


    return 0;

}

