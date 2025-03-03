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
#include "Components/DecompositionPRM.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"

namespace Antipatrea {

    void DecompositionPRM::Construct(const double tmax) {
        Timer::Clock clk;
        Timer::Clock t1;
        Timer::Start(clk);

        m_proximityAuxCfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        GetSimulator()->GetCfg(&m_proximityAuxCfg[0]);

        auto keyInit = AddRegion(&m_proximityAuxCfg[0]);
        AddGoals();
        SetInitialAndGoalsPairs(keyInit);

        auto sim = GetSimulator();
        auto scene = sim->GetScene();
        auto mmCfg = sim->GetCfgAllocator()->New();


        Timer::Start(t1);
//      setting the goal
        auto nr = GetGraph()->GetNrVertices();
        auto goals = GetProblem()->GetGoals();
        auto matrix = GetSimulator()->GetScene()->m_mmMatrix;
        auto dis = GetSimulator()->GetScene()->m_mmPredictedPaths;

        // setting the start
        auto r = dynamic_cast<Region *>(m_graph.GetVertexByIndex(0));
        int idx = -1;
        for (int i = 0; i < GetSimulator()->GetScene()->m_mmCors.size(); ++i){
            if (GetSimulator()->GetScene()->m_mmCors[i][0] == r->GetCfg()[0] && GetSimulator()->GetScene()->m_mmCors[i][1] == r->GetCfg()[1]){
                idx = i;
                break;
            }
        }
        r->id = idx;
        std::vector<double> subVector(matrix.begin() + idx * nr, matrix.begin() + (idx + 1) * nr);
        std::vector<std::vector<std::vector<double>>> subDisVector(dis.begin() + idx * nr,dis.begin() + (idx + 1) * nr);

        for(int i = 0; i < subVector.size(); ++i){
            if (i != idx){
                r->m_predictedDistancesToGoals.push_back(subVector[i]);
                r->m_predictedToGoalID.push_back(i);
                r->m_predictCorToGoals.push_back(subDisVector[i][0]);
            }
        }

        // setting the goal
        for (int i = 0; i < goals->size(); ++i){
            auto regGoal = (*goals)[i];
            idx = -1;
            for (int j = 0; j < GetSimulator()->GetScene()->m_mmCors.size(); ++j){
                if (GetSimulator()->GetScene()->m_mmCors[j][0] == regGoal->GetCfg()[0] && GetSimulator()->GetScene()->m_mmCors[j][1] == regGoal->GetCfg()[1]){
                    idx = j;
                    break;
                }
            }
            regGoal->id = idx;
            std::vector<double> subGoalVector(matrix.begin() + idx * nr, matrix.begin() + (idx + 1) * nr);
            std::vector<std::vector<std::vector<double>>> subGoalDisVector(dis.begin() + idx * nr,dis.begin() + (idx + 1) * nr);

            for(int j = 0; j < subVector.size(); ++j){
                if (j != r->id){
                    regGoal->m_predictedDistancesToGoals.push_back(subGoalVector[j]);
                    regGoal->m_predictedToGoalID.push_back(j);
                    if (!subGoalDisVector[j].empty())
                        regGoal->m_predictCorToGoals.push_back(subGoalDisVector[j][0]);
                    else{
                        std::vector<double> path;
                        regGoal->m_predictCorToGoals.push_back(path);
                    }
                }
            }
        }

        for (int i = 0; i < scene->m_mmPredictedPaths.size(); ++i){

            if (scene->m_mmPredictedPaths.size() >= 50 && RandomUniformReal() >= 0.5 )
                continue;
            else if (!scene->m_mmPredictedPaths[i].empty()){
                auto path = &scene->m_mmPredictedPaths[i][0];
                for (int m = 0; m < path->size(); m += 4) {
                    sim->SampleCfg(mmCfg);
                    int mmCount = 0;
                    for (int k = 0; k < 10; ++k) {
                        RandomPointInsideSphere(2, &((*path)[m]), 4.0, mmCfg);
                        if (sim->IsValidCfg(mmCfg)) {
                            AddRegion(mmCfg);
                            ConnectRegions(tmax - Timer::Elapsed(clk));
                            ++mmCount;
                            Stats::GetSingleton()->AddValue("NrRegions", 1);
                            if (++mmCount >= 2)
                                break;
                        }

                    }
                }
            }

        }



        PathsToGoals();

        Stats::GetSingleton()->AddValue("PRMForPredictPath", Timer::Elapsed(t1));

    }

    Id DecompositionPRM::LocateRegion(const double cfg[]) {
        m_proximityAuxCfg.resize(GetSimulator()->GetCfgAllocator()->GetDim());
        GetSimulator()->GetCfgAllocator()->Copy(&m_proximityAuxCfg[0], cfg);

        ProximityQuery<Id> query;

        query.SetKey(Constants::ID_UNDEFINED);
        return m_proximity.Neighbor(query);
    }

    void DecompositionPRM::AddGoals(void) {
        Decomposition::AddGoals();
        for (auto &goal: *(GetProblem()->GetGoals())) {
            m_proximity.AddKey(goal->GetKey());
            m_regionsToBeConnected.insert(goal->GetKey());
        }
    }

    Id DecompositionPRM::AddRegion(const double cfg[]) {
        auto r = new Region();
        double p[3];

        r->SetKey(GetGraph()->GetNrVertices());
        r->SetCfg(GetSimulator()->GetCfgAllocator()->Copy(cfg));

        GetSimulator()->GetPositionCfg(cfg, p);
        r->SetClearance(GetSimulator()->ClearancePosition(p));
        UpdateMinMaxRegionClearances(r->GetClearance());

        GetGraph()->AddVertex(r);
        m_proximity.AddKey(r->GetKey());
        m_regionsToBeConnected.insert(r->GetKey());

        return r->GetKey();
    }

    int DecompositionPRM::AddRegions(const int nrCfgs, const double tmax) {
        auto sim = GetSimulator();
        auto cfg = sim->GetCfgAllocator()->New();
        int count = 0;
        Timer::Clock clk;
        Timer::Start(clk);
        while (count < nrCfgs && Timer::Elapsed(clk) < tmax && GetGraph()->GetNrVertices() < GetMaxNrVertices()) {
            sim->SampleCfg(cfg);
            if (sim->IsValidCfg(cfg)) {
                ++count;
                AddRegion(cfg);
                cfg = sim->GetCfgAllocator()->New();
            }
        }
        sim->GetCfgAllocator()->Delete(cfg);

        return count;
    }

    int DecompositionPRM::ConnectRegions(const double tmax) {
        Timer::Clock clk;
        Timer::Start(clk);

        while ((!AreInitialAndGoalsConnected() || !GetStopWhenConnected()) &&
               m_regionsToBeConnected.empty() == false)
            ConnectRegion(*(m_regionsToBeConnected.begin()));

        return m_regionsToBeConnected.size();
    }

    void DecompositionPRM::ConnectRegion(const Id key) {
        ProximityQuery<Id> query;
        ProximityResults<Id> res;

        query.SetNrNeighbors(GetNrNeighbors());
        query.SetKey(key);
        m_proximity.Neighbors(query, res);

        const int n = res.GetNrResults();
        for (int i = 0; i < n; ++i)
            GenerateEdge(key, res.GetKey(i), res.GetDistance(i));

        m_regionsToBeConnected.erase(key);
    }

    bool DecompositionPRM::GenerateEdge(const Id key1, const Id key2, const double d) {
        auto sim = GetSimulator();

        if (key1 == key2 ||
            (RandomUniformReal() > GetProbabilityAllowCycles() && GetGraph()->AreVerticesPathConnected(key1, key2)) ||
            m_attempts.find(std::make_pair(key1, key2)) != m_attempts.end())
            return false;

        m_attempts.insert(std::make_pair(key1, key2));
        m_attempts.insert(std::make_pair(key2, key1));

        auto r1 = dynamic_cast<Region *>(GetGraph()->GetVertex(key1));
        auto r2 = dynamic_cast<Region *>(GetGraph()->GetVertex(key2));

        if (d > GetOneStepDistance()) {
            auto dt = AdjustTimeStep(GetOneStepDistance() / d);

            if (!sim->IsValidPathCfgs(r1->GetCfg(), r2->GetCfg(), dt, dt, 1.0 - dt))
                return false;
        }

        DecompositionEdge *e1 = new DecompositionEdge();
        e1->SetFromToVertexKeys(key1, key2);
        e1->SetValues(*sim, *r1, *r2);
        GetGraph()->AddEdge(e1);

        DecompositionEdge *e2 = new DecompositionEdge();
        e2->SetFromToVertexKeys(key2, key1);
        e2->CopyValuesFrom(*e1);
        GetGraph()->AddEdge(e2);

        return true;
    }

    double DecompositionPRM::ProximityDistanceFn(const Id key1, const Id key2, DecompositionPRM *prm) {
        auto v1 = prm->GetGraph()->GetVertex(key1);
        auto v2 = prm->GetGraph()->GetVertex(key2);
        const double *cfg1 = v1 ? dynamic_cast<Region *>(v1)->GetCfg() : &(prm->m_proximityAuxCfg[0]);
        const double *cfg2 = v2 ? dynamic_cast<Region *>(v2)->GetCfg() : &(prm->m_proximityAuxCfg[0]);

        return prm->GetSimulator()->DistanceCfgs(cfg1, cfg2);
    }
}
