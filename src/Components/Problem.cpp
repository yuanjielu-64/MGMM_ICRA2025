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
#include "Components/Problem.hpp"
#include "Components/Constants.hpp"
#include "Components/RegionBox2D.hpp"
#include "Components/RegionPolygon2D.hpp"
#include "Components/RegionBox3D.hpp"
#include "Utils/Stats.hpp"
#include <iomanip>

namespace Antipatrea {

    void Problem::SetupFromParams(Params &params) {
        Component::SetupFromParams(params);

        RegionGeometric *r;
        double bounds[2];

        const int nrGoals = params.GetValueAsInt(Constants::KW_NrGoals, 0);
        std::string str;
        for (int i = 0; i < nrGoals; ++i) {
            r = NULL;
            str = ((std::string) Constants::KW_Goal) + std::to_string(i);
            auto data = params.GetData(str.c_str());
            if (data && data->m_params) {
                if (data->m_params->HasParam(Constants::KW_Polygon2D))
                    r = new RegionPolygon2D();
                else if (data->m_params->HasParam(Constants::KW_Box2D))
                    r = new RegionBox2D();
                else if (data->m_params->HasParam(Constants::KW_Box3D))
                    r = new RegionBox3D();
                if (r != NULL) {
                    r->SetFlags(RemoveAndAddFlags(r->GetFlags(), Region::STATUS_REGULAR, Region::STATUS_GOAL));
                    r->SetupFromParams(*(data->m_params));
                    m_goals.push_back(r);

                    bounds[0] = 0.0;
                    bounds[1] = INFINITY;
                    data->m_params->GetValuesAsDoubles(Constants::KW_TimeBounds, bounds, 2);
                    m_timeBounds.push_back(bounds[0]);
                    m_timeBounds.push_back(bounds[1]);
                }
            }
        }

        Logger::m_out << "Problem::SetupFromParams...number of goals: " << m_goals.size() << std::endl;
    }

    int Problem::LocateGoal(const double p[]) const {
        for (int i = m_goals.size() - 1; i >= 0; --i)
            if (m_goals[i]->IsPointInside(p))
                return i;
        return Constants::ID_UNDEFINED;
    }

    void Problem::DrawGoals(void) {
        double pos[3];
        char msg[300];
        char msg1[300];

        pos[2] = 0.0;
        for (int i = m_goals.size() - 1; i >= 0; --i) {
            GDrawColor(m_goals[i]->GetColor());
            if (m_gShowIds[i] >= 0)
                GDrawColor(0.2, 0.8, 0.7);
            else
                GDrawColor(1, 0, 0);

            GDrawPushTransformation();
            GDrawMultTrans(0, 0, 0.1);
            m_goals[i]->DrawShape();
            GDrawPopTransformation();

            GDrawColor(0.0, 0.0, 0.0);
            m_goals[i]->GetDrawTextPos(pos);

            sprintf(msg, "G%d", i);

            double totalTime = 0.0;
            totalTime = Stats::GetSingleton()->GetValue("TimeSolve")
                        + Stats::GetSingleton()->GetValue("PRMForPredictPath");

            sprintf(msg1, "Nr. of Samples: %d;  Planning Time: %.2f seconds", (int) Stats::GetSingleton()->GetValue("NrRegions"), totalTime);

            GDrawString3D(msg1, 20, 55, pos[2], true);
            GDrawString3D(msg, pos[0], pos[1], pos[2], true);
        }
    }
}
