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

#ifndef Antipatrea__MPTreeVertex_HPP_
#define Antipatrea__MPTreeVertex_HPP_

#include "Components/Group.hpp"
#include "Utils/Constants.hpp"
#include "Utils/TreeVertex.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

namespace Antipatrea {
    class MPTreeVertex
            : public TreeVertex<Id>, public GroupContainer {
    public:
        MPTreeVertex(void)
                : TreeVertex<Id>(), GroupContainer(), m_state(NULL), m_goal(Constants::ID_UNDEFINED),
                  m_rid(Constants::ID_UNDEFINED), m_time(0.0), m_nextWaypt(Constants::ID_UNDEFINED) {
        }

        virtual ~MPTreeVertex(void) {
            if (m_state)
                delete[] m_state;
        }

        virtual const double *GetState(void) const {
            return m_state;
        }

        virtual double *GetState(void) {
            return m_state;
        }

        virtual int GetReachedGoal(void) const {
            return m_goal;
        }

        virtual Id GetRegion(void) const {
            return m_rid;
        }

        virtual double GetTime(void) const {
            return m_time;
        }

        virtual int GetNextWaypt(void) const {
            return m_nextWaypt;
        }

        virtual void SetState(double s[]) {
            m_state = s;
        }

        virtual void SetReachedGoal(const int gid) {
            m_goal = gid;
        }

        virtual void SetNextGoal(const int gid) {
            m_nextGoal = gid;
        }

        virtual int GetNextGoal(){
            return m_nextGoal;
        }

        virtual void SetRegion(const Id rid) {
            m_rid = rid;
        }

        virtual void SetInitialPredictedPath(std::vector<std::vector<std::vector<double>>> &path, int index, int nrGoal) {
            int nrPoints = nrGoal + 1;
            int idx =  index * (nrGoal + 1);
            std::vector<std::vector<std::vector<double>>> subVector(path.begin() + idx, path.begin() + idx + nrPoints);
            for(auto & i : subVector)
                ppaths.push_back(i);
        }

        virtual void RemoveFirstPath() {
            paths.erase(paths.begin(), paths.begin() + 2);
        }

        virtual void SetPredictPath(const std::vector<double> &path){
            paths = path;
        }

        virtual std::vector<double> GetPredictPath() {
            return paths;
        }

        virtual void SetPredictPPath(const std::vector<std::vector<std::vector<double>>> &path){
            ppaths = path;
        }

        virtual std::vector<std::vector<std::vector<double>>> GetPredictPPath() {
            return ppaths;
        }

        virtual void SetTime(const double t) {
            m_time = t;
        }

        virtual void SetNextWaypt(const int waypt) {
            m_nextWaypt = waypt;
        }

    protected:
        double *m_state;
        int m_goal;
        int m_nextGoal;
        Id m_rid;
        double m_time;
        int m_nextWaypt;

        std::vector<double> paths;
        std::vector<std::vector<std::vector<double>>> ppaths;
    };
}

#endif



