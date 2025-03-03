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

#ifndef Antipatrea__Dromos_HPP_
#define Antipatrea__Dromos_HPP_

#include "Components/GroupKeyTour.hpp"
#include "Components/GroupSelector.hpp"
#include "Components/TourGenerator.hpp"
#include "Planners/MPTree.hpp"

namespace Antipatrea {
    class Dromos
            : public MPTree, public GroupSelectorContainer, public TourGeneratorContainer {
    public:
        Dromos(void)
                : MPTree(), GroupSelectorContainer(), TourGeneratorContainer() {
        }

        virtual ~Dromos(void) {
        }

        virtual bool Solve(const int nrIters, const double tmax, bool &canBeSolved);

    protected:
        virtual GroupKey *NewGroupKey(void) const {
            return new GroupKeyTour();
        }

        virtual bool CompleteGroup(Group &group, MPTreeVertex &v);

    };

    ClassContainer(Dromos, m_dromos);
}

#endif
