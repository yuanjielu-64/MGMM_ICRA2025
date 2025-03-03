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

#ifndef Antipatrea__Sequential_HPP_
#define Antipatrea__Sequential_HPP_

#include "Components/GroupKeyTour.hpp"
#include "Components/GroupSelector.hpp"
#include "Components/TourGenerator.hpp"
#include "Planners/MPTree.hpp"

namespace Antipatrea
{
class Sequential
    : public MPTree
    , public GroupSelectorContainer
    , public TourGeneratorContainer
{
public:
  Sequential(void)
      : MPTree()
      , GroupSelectorContainer()
      , TourGeneratorContainer()
      , m_index(0)
  {
      m_enforceOrder = true;
  }

  virtual ~Sequential(void)
  {
  }
    
    virtual bool Solve(const int nrIters, const double tmax, bool &canBeSolved);
    
protected:
    virtual void AddVertex(MPTreeVertex * const v);
    
    virtual GroupKey *NewGroupKey(void) const
    {
	return new GroupKeyTour();
    }
    
    virtual bool CompleteGroup(Group &group, MPTreeVertex &v);

    Groups m_seqGroups;
    std::vector<int> m_order;
    int m_index;
    
    
};

ClassContainer(Sequential, m_sequential);
}

#endif
