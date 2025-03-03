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

#ifndef Antipatrea__Groups_HPP_
#define Antipatrea__Groups_HPP_

#include "Components/Group.hpp"
#include "Components/GroupKey.hpp"
#include "Components/GroupKeyTour.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Misc.hpp"

namespace Antipatrea {

    class Groups {
    public:
        Groups(void) : m_sumWeights(0.0) {
        }

        virtual ~Groups(void) {
        }

        virtual void Clear(void) {
            DeleteItems<Group *>(m_groups);
            m_groups.clear();
            m_sumWeights = 0.0;
        }

        virtual int GetNrGroups(void) const {
            return m_groups.size();
        }

        virtual const Group *GetGroup(const int i) const {
            return m_groups[i];
        }

        virtual Group *GetGroup(const int i) {
            return m_groups[i];
        }

        virtual void AddGroup(Group *const group) {
            m_groups.push_back(group);
            m_sumWeights += group->GetWeight();
        }

        virtual Group *FindGroup(const GroupKey &key) const;

        virtual double GetSumWeights(void) const {
            return m_sumWeights;
        }

        virtual void SetSumWeights(const double w) {
            m_sumWeights = w;
        }

        virtual int GetSize(){
            return m_groups.size();
        }

        virtual void removeAllGroups(){
            std::vector<Group *> new_groups;
            auto group = dynamic_cast<GroupKeyTour *> (m_groups[GetSize() - 1]->GetKey());
            auto nrGoal = group->GetTour()->m_goalOrder.size();

            for (auto & m_group : m_groups){
                auto g = dynamic_cast<GroupKeyTour *> (m_group->GetKey());
                auto ng = g->GetTour()->m_goalOrder.size();
                if (ng == nrGoal)
                    new_groups.push_back(m_group);
            }

            m_groups.clear();
            m_groups = new_groups;

        }

        virtual void UpdateWeightGroup(Group &group, const double w) {
            SetSumWeights(GetSumWeights() - group.GetWeight() + w);
            group.SetWeight(w);
        }

        virtual void UpdateSumWeights(void);

    protected:
        std::vector<Group *> m_groups;
        double m_sumWeights;
    };

    ClassContainer(Groups, m_groups);
}

#endif
