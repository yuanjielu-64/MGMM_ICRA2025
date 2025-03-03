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
#include "Planners/RRT.hpp"
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"
#include <iostream>

namespace Antipatrea {

    bool RRT::Solve(const int nrIters, const double tmax, bool &canBeSolved) {
        Timer::Clock clk;
        std::vector<double> pos;
        Group *group;

        Timer::Start(clk);

        if (m_tree.GetNrVertices() == 0) {
            if (Start() == false) {
                canBeSolved = false;
                return false;
            }

            if (m_tree.GetNrVertices() >= 1) {
                auto vRoot = dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(0));
                auto keyRoot = dynamic_cast<GroupKeyTour *>(vRoot->GetGroup()->GetKey());
                auto ord = &(keyRoot->GetTour()->m_order);
                for (int i = 1; i < (int) ord->size(); ++i)
                    m_order.push_back(keyRoot->GetUnreachedGoalsKey()->GetUnreachedGoals()->operator[]((*ord)[i] - 1));

                Logger::m_out << "MAIN TOUR:";
                for (int i = 0; i < (int) m_order.size(); ++i)
                    Logger::m_out << m_order[i] << " ";
                Logger::m_out << std::endl;

                m_index = 0;
                if (m_order.size() > 0)
                    m_gidCurr = m_order[0];

            }
        }

        std::vector<double> aux;

        aux.resize(GetSimulator()->GetPositionAllocator()->GetDim());

        pos.resize(GetSimulator()->GetPositionAllocator()->GetDim());
        for (int i = 0; i < nrIters && Timer::Elapsed(clk) < tmax && !IsSolved(); ++i) {
            if (RandomUniformReal() <= 0.05 && m_index >= 0 && m_index < m_order.size())
                GetProblem()->GetGoals()->operator[](m_order[m_index])->SamplePointInside(&pos[0]);
            else
                GetSimulator()->SamplePosition(&pos[0]);

            double dmin = INFINITY;
            int best = -1;

            for (int j = m_use_vids.size() - 1; j >= 0 && j > (((int) m_use_vids.size()) - 5000); --j) {
                GetSimulator()->GetPositionState(
                        dynamic_cast<MPTreeVertex *>(m_tree.GetVertex(m_use_vids[j]))->GetState(), &aux[0]);
                const double d = GetSimulator()->DistancePositions(&aux[0], &pos[0]);
                if (d < dmin) {
                    dmin = d;
                    best = j;

                    if (RandomUniformReal() < 0.0005)
                        break;
                }
            }

            if (best >= 0) {
//		Logger::m_out << " extending from " << m_use_vids[best] << " out of " << m_use_vids.size() << std::endl;
                ExtendFrom(m_use_vids[best], &pos[0]);
            }

        }

        return IsSolved();
    }

    bool RRT::CompleteGroup(Group &group, MPTreeVertex &v) {
        Timer::Clock clk;
        Timer::Start(clk);

        auto tkey = dynamic_cast<GroupKeyTour *>(group.GetKey());

        if (tkey == NULL)
            return false;

        auto goals = GetProblem()->GetGoals();

        const double dur = Algebra::PointDistance(2, v.GetState(), (*goals)[0]->GetCfg());

        if (dur < Constants::EPSILON)
            group.SetWeight(Constants::GROUP_MAX_WEIGHT);
        else
            group.SetWeight(10000000000.0 / dur);

        Stats::GetSingleton()->AddValue("TimeCompleteGroup", Timer::Elapsed(clk));

        return true;

    }

    void RRT::AddVertex(MPTreeVertex *const v) {

        MPTree::AddVertex(v);

        if (m_index >= 0 && m_index < m_order.size() && m_order[m_index] == v->GetReachedGoal()) {
            //goal reached
            m_use_vids.clear();

            ++m_index;
            if (m_index < m_order.size())
                m_gidCurr = m_order[m_index];
            else
                m_vidSolved = v->GetKey();

            Logger::m_out << "reached " << m_index << "/" << m_order.size() << " goals" << std::endl;
        }
        m_use_vids.push_back(v->GetKey());

    }
}

