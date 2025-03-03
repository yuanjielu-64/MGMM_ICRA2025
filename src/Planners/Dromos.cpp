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
#include "Planners/Dromos.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include "Components/predictLKH.h"
#include "Components/DecompositionPRM.hpp"
#include <iostream>

namespace Antipatrea {

    bool Dromos::Solve(const int nrIters, const double tmax, bool &canBeSolved) {
        Timer::Clock clk;
        std::vector<double> pos;
        Group *group;

        Timer::Start(clk);

        if (m_tree.GetNrVertices() == 0) {
            Logger::m_out << "DROMOS" << std::endl;
            if (!Start()) {
                canBeSolved = false;
                return false;
            }
        }

        std::vector<double> temp_pos;
        auto dims = GetSimulator()->GetPositionAllocator()->GetDim();

        temp_pos.resize(dims);

        pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());

        for (int i = 0; i < nrIters && Timer::Elapsed(clk) < tmax  && !IsSolved(); ++i) {
            Timer::Clock clk1;
            Timer::Start(clk1);

            if (m_shouldWait)
                ExtendWait();
            else {
                GetGroupSelector()->SetGroups(&m_groups);
                group = GetGroupSelector()->SelectGroup();
                const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();

                GetSimulator()->SamplePosition(&temp_pos[0]);
                auto vid = SelectVertex(*group, &temp_pos[0]);
                auto v = dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(vid));

                auto vCfg = v->GetState();
                auto tkey = dynamic_cast<GroupKeyTour *>(group->GetKey());

                auto kgoals = tkey->GetUnreachedGoalsKey();
                auto unreached = kgoals->GetUnreachedGoals();

                if (!v->GetPredictPath().empty()){
                    auto a = v->GetPredictPath();

                    int count = 0;
                    double r = 1;
                    while(true){

                        RandomPointInsideSphere(dimPos, &a[0], r, &pos[0]);
                        double d = Algebra::PointDistance(2, vCfg, &pos[0]);
                        auto dt = AdjustTimeStep(0.25 / d);
                        if (GetSimulator()->IsValidPathCfgs(&v->GetState()[0], &pos[0], dt, dt, 1.0 - dt)){
                            v->RemoveFirstPath();
                            break;
                        }else
                            count++;

                        if (count >= 50){
                            v->RemoveFirstPath();
                            break;
                        }
                    }
                }else
                    GetSimulator()->SamplePosition(&pos[0]);

                Stats::GetSingleton()->AddValue("TimeForXXX", Timer::Elapsed(clk1));

                int flag = -1;
                for (int j : *unreached){
                    if (GetSimulator()->DistancePositions(&vCfg[0], (*GetProblem()->GetGoals())[j]->GetCfg()) <= 3){
                        ExtendFrom(vid, (*GetProblem()->GetGoals())[j]->GetCfg());
                        flag = 0;
                        break;
                    }
                }

                if (flag == -1)
                    ExtendFrom(vid, &pos[0]);

                m_groups.UpdateWeightGroup(*group, group->GetWeight() * GetDiscountSelect());

            }
        }

        return IsSolved();
    }

    bool Dromos::CompleteGroup(Group &group, MPTreeVertex &v) {
        Timer::Clock clk;
        Timer::Start(clk);

        auto tkey = dynamic_cast<GroupKeyTour *>(group.GetKey());

        if (tkey == nullptr)
            return false;

        bool ok = false;
        auto tgen = GetTourGenerator();
        auto kreg = tkey->GetRegionKey();
        auto kgoals = tkey->GetUnreachedGoalsKey();
        auto unreached = kgoals->GetUnreachedGoals();
        auto ng = unreached->size();
        auto decomp = GetDecomposition();
        auto goals = GetProblem()->GetGoals();
        double cost;

        auto sim = GetSimulator();
        auto scene = sim->GetScene();
        auto matrix = scene->m_mmMatrix;

        tgen->SetNrSites(1 + ng);
        tgen->SetStartTime(tkey->GetTour()->GetStartTime());
        tgen->SetBounds(0, 0.0, INFINITY);
        tgen->SetDuration(0, 0, 0.0);

        auto regStart = dynamic_cast<Region *>(decomp->GetGraph()->GetVertex(kreg->GetId()));

        for (int i = 0; i < ng; ++i)
            tgen->SetBounds(i + 1, 0, INFINITY);

        if (v.GetParent()) {
            auto parent = dynamic_cast<MPTreeVertex *>(v.GetParent());
            auto pkey = dynamic_cast<GroupKeyTour *>(parent->GetGroup()->GetKey());
            // new group but no new goal is reached
            if (kgoals->SameContent(*(pkey->GetUnreachedGoalsKey()))) {

                regStart = dynamic_cast<Region *>(decomp->GetGraph()->GetVertex(kreg->GetId()));

                //setting prediction values from start to each goal
                for (int i = 0; i < ng; ++i) {
                    tgen->SetDuration(i + 1, i + 1, 0.0);
                    tgen->SetBounds(i + 1, 0, INFINITY);

                    auto data = regStart->GetPathDataToGoals();
                    cost = (*data)[(*unreached)[i]]->m_cost;
                    if (cost != 0){
                        tgen->SetDuration(0, i + 1, cost);
                        tgen->SetDuration(i + 1, 0, cost);
                    }else{
                        cost = Algebra::PointDistance(2, v.GetState(), (*goals)[(*unreached)[i]]->GetCfg());
                        //cost = regStart->m_predictedDistancesToGoals[i];
                        tgen->SetDuration(0, i + 1, cost);
                        tgen->SetDuration(i + 1, 0, cost);
                    }
                }

                //setting prediction values from each goal to each goal
                for (int i = 0; i < ng; ++i) {
                    auto regGoal1 = (*goals)[(*unreached)[i]];
                    auto data = (*goals)[(*unreached)[i]]->GetPathDataToGoals();

                    for (int j = i + 1; j < ng; ++j) {
                        auto regGoal2 = (*goals)[(*unreached)[j]];
                        cost = (*data)[(*unreached)[j]]->m_cost;
                        if (cost != 0){
                            tgen->SetDuration(i + 1, j + 1, cost);
                            tgen->SetDuration(j + 1, i + 1, cost);
                        }else{
                            cost = Algebra::PointDistance(2, v.GetState(), (*goals)[(*unreached)[i]]->GetCfg());
                            //cost = regGoal1->m_predictedDistancesToGoals[j];
                            tgen->SetDuration(i + 1, j + 1, cost);
                            tgen->SetDuration(j + 1, i + 1, cost);
                        }
                    }
                }

                tkey->GetTour()->m_order = pkey->GetTour()->m_order;
                tkey->GetTour()->m_goalOrder = pkey->GetTour()->m_goalOrder;
                ok = tgen->FromOrderToTimes(*(tkey->GetTour()));

            }else{

                int reachGoal = v.GetReachedGoal();
                if (reachGoal == 2)
                    auto a = 10;

                auto regGoal = (*goals)[reachGoal];
                //setting prediction values from start to each goal
                for (int i = 0; i < ng; ++i) {
                    tgen->SetDuration(i + 1, i + 1, 0.0);
                    tgen->SetBounds(i + 1, 0, INFINITY);

                    auto data = regGoal->GetPathDataToGoals();
                    cost = (*data)[(*unreached)[i]]->m_cost;
                    if (cost != 0){
                        tgen->SetDuration(0, i + 1, cost);
                        tgen->SetDuration(i + 1, 0, cost);
                    }else{
                        cost = regGoal->m_predictedDistancesToGoals[i];
                        tgen->SetDuration(0, i + 1, cost);
                        tgen->SetDuration(i + 1, 0, cost);
                    }
                }

                //setting prediction values from each goal to each goal
                for (int i = 0; i < ng; ++i) {
                    auto regGoal1 = (*goals)[(*unreached)[i]];
                    auto data = (*goals)[(*unreached)[i]]->GetPathDataToGoals();

                    for (int j = i + 1; j < ng; ++j) {
                        auto regGoal2 = (*goals)[(*unreached)[j]];
                        cost = (*data)[(*unreached)[j]]->m_cost;
                        if (cost != 0){
                            tgen->SetDuration(i + 1, j + 1, cost);
                            tgen->SetDuration(j + 1, i + 1, cost);
                        }else{
                            cost = regGoal1->m_predictedDistancesToGoals[j];
                            tgen->SetDuration(i + 1, j + 1, cost);
                            tgen->SetDuration(j + 1, i + 1, cost);
                        }
                    }
                }

                // new group but a new goal is reached
                tkey->GetTour()->m_goalOrder = pkey->GetTour()->m_goalOrder;
                auto itr = std::find(tkey->GetTour()->m_goalOrder.begin(), tkey->GetTour()->m_goalOrder.end(), reachGoal);
                int idx = std::distance(tkey->GetTour()->m_goalOrder.begin(), itr);

                if (idx == 0) {
                    tkey->GetTour()->m_goalOrder.erase(tkey->GetTour()->m_goalOrder.begin() + idx);
                    tkey->GetTour()->m_order.push_back(0);
                    for (int i = 0; i < tkey->GetTour()->m_goalOrder.size(); ++i){
                        itr = std::find((*unreached).begin(), (*unreached).end(), tkey->GetTour()->m_goalOrder[i]);
                        idx = std::distance((*unreached).begin(), itr) + 1;
                        tkey->GetTour()->m_order.push_back(idx);
                    }

                    std::vector<double> aa;
                    for (int i = 1; i<tkey->GetTour()->m_order.size(); ++i)
                        aa.push_back((*unreached)[tkey->GetTour()->m_order[i] - 1]);

                    ok = tgen->FromOrderToTimes(*(tkey->GetTour()));
                }else{
                    ok = tgen->GenerateTour(*(tkey->GetTour()));
                    tkey->GetTour()->m_goalOrder.clear();
                    for (int i=1; i<tkey->GetTour()->m_order.size(); ++i)
                        tkey->GetTour()->m_goalOrder.push_back((*unreached)[tkey->GetTour()->m_order[i] - 1]);

                    Stats::GetSingleton()->AddValue("NrCallsGenerateTour", 1);
                }

                if(tkey->GetTour()->m_order.size() > 1)
                {
                    auto xgid = (*unreached)[tkey->GetTour()->m_order[1] - 1];
                    auto xgdata = (*regGoal->GetPathDataToGoals())[xgid];

                    if (xgdata->m_cost != 0 && xgdata->m_pts.size() != 0){
                        const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();
                        group.m_guide.clear();
                        RegularizePointsAlongPath(xgdata->m_pts.size() / dimPos, &xgdata->m_pts[0], 3.0, dimPos, group.m_guide);

                        v.SetPredictPath(group.m_guide);
                    }else{
                        auto xg = regGoal->m_predictCorToGoals[xgid];
                        const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();
                        group.m_guide.clear();
                        RegularizePointsAlongPath(xgdata->m_pts.size() / dimPos, &xg[0], 1.0, dimPos, group.m_guide);
                        v.SetPredictPath(group.m_guide);
                    }
                }
                Stats::GetSingleton()->AddValue("NrCallsGenerateTour", 1);
            }

        }else{
            // initial TSP by the predictive length
            regStart = dynamic_cast<Region *>(decomp->GetGraph()->GetVertex(kreg->GetId()));

            //setting prediction values from start to each goal
            for (int i = 0; i < ng; ++i) {
                tgen->SetDuration(i + 1, i + 1, 0.0);
                tgen->SetBounds(i + 1, 0, INFINITY);

                auto data = regStart->GetPathDataToGoals();
                cost = (*data)[(*unreached)[i]]->m_cost;
                if (cost != 0){
                    tgen->SetDuration(0, i + 1, cost);
                    tgen->SetDuration(i + 1, 0, cost);
                }else{
                    cost = regStart->m_predictedDistancesToGoals[i];
                    tgen->SetDuration(0, i + 1, cost);
                    tgen->SetDuration(i + 1, 0, cost);
                }
            }

            //setting prediction values from each goal to each goal
            for (int i = 0; i < ng; ++i) {
                auto regGoal1 = (*goals)[(*unreached)[i]];
                auto data = (*goals)[(*unreached)[i]]->GetPathDataToGoals();

                for (int j = i + 1; j < ng; ++j) {
                    auto regGoal2 = (*goals)[(*unreached)[j]];
                    cost = (*data)[(*unreached)[j]]->m_cost;
                    if (cost != 0){
                        tgen->SetDuration(i + 1, j + 1, cost);
                        tgen->SetDuration(j + 1, i + 1, cost);
                    }else{
                        cost = regGoal1->m_predictedDistancesToGoals[j];
                        tgen->SetDuration(i + 1, j + 1, cost);
                        tgen->SetDuration(j + 1, i + 1, cost);
                    }
                }
            }

            ok = tgen->GenerateTour(*(tkey->GetTour()));
            for (int i = 1; i < tkey->GetTour()->m_order.size(); ++i)
                tkey->GetTour()->m_goalOrder.push_back((*unreached)[tkey->GetTour()->m_order[i] - 1]);

            if(tkey->GetTour()->m_order.size() > 1)
            {
                auto xgid = (*unreached)[tkey->GetTour()->m_order[1] - 1];
                auto xgdata = (*regStart->GetPathDataToGoals())[xgid];

                if (xgdata->m_cost != 0 && xgdata->m_pts.size() != 0){
                    const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();
                    group.m_guide.clear();
                    RegularizePointsAlongPath(xgdata->m_pts.size() / dimPos, &xgdata->m_pts[0], 3.0, dimPos, group.m_guide);

                    v.SetPredictPath(group.m_guide);
                }else{
                    auto xg = regStart->m_predictCorToGoals[xgid];
                    const int dimPos = GetSimulator()->GetPositionAllocator()->GetDim();
                    group.m_guide.clear();
                    RegularizePointsAlongPath(xgdata->m_pts.size() / dimPos, &xg[0], 3.0, dimPos, group.m_guide);
                    v.SetPredictPath(group.m_guide);
                }
            }
            Stats::GetSingleton()->AddValue("NrCallsGenerateTour", 1);
        }

        if (ok){
            const double dur = tkey->GetTour()->GetEndTime() - tkey->GetTour()->GetStartTime();

            if (dur < Constants::EPSILON)
                group.SetWeight(Constants::GROUP_MAX_WEIGHT);
            else
                group.SetWeight(10000000.0 / dur);

            Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

            return true;
        }

        Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

        return false;

    }
}

