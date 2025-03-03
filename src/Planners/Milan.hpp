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

#ifndef Antipatrea__Milan_HPP_
#define Antipatrea__Milan_HPP_

#include "Planners/MPTree.hpp"
#include "Components/GroupSelector.hpp"
#include "Components/GroupKeyMulti.hpp"
#include "Components/GroupKeyRegion.hpp"
#include "Components/GroupKeyUnreachedGoals.hpp"

namespace Antipatrea
{
class Milan : public MPTree,
              public GroupSelectorContainer
{
  public:
    Milan(void) : MPTree(),
                  GroupSelectorContainer()
    {
    }

    virtual ~Milan(void)
    {
    }

    virtual bool Solve(const int nrIters, const double tmax, bool & canBeSolved);

  protected:
    virtual GroupKey *NewGroupKey(void) const
    {
        auto multi = new GroupKeyMulti();
        multi->GetKeys()->push_back(new GroupKeyRegion());
        multi->GetKeys()->push_back(new GroupKeyUnreachedGoals());

        return multi;
    }
};

ClassContainer(Milan, m_milan);
}

#endif
