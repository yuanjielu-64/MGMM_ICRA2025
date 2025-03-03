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

#ifndef Antipatrea__CreateScenesConstants_HPP_
#define Antipatrea__CreateScenesConstants_HPP_

namespace Antipatrea {

    namespace Constants {

        const char KW_ShapeMinNrVertices[] = "ShapeMinNrVertices";
        const char KW_ShapeMaxNrVertices[] = "ShapeMaxNrVertices";
        const char KW_ShapeMinDims[] = "ShapeMinDims";
        const char KW_ShapeMaxDims[] = "ShapeMaxDims";
        const char KW_ShapeThickness[] = "ShapeThickness";
        const char KW_ShapeProbabilityL[] = "ShapeProbabilityL";
        const char KW_ShapeProbabilityU[] = "ShapeProbabilityU";
        const char KW_ShapeProbabilityRectangle[] = "ShapeProbabilityRectangle";

        const int SHAPE_MIN_NR_VERTICES = 3;
        const int SHAPE_MAX_NR_VERTICES = 8;
        const double SHAPE_MIN_DIM_X = 4.0;
        const double SHAPE_MIN_DIM_Y = 4.0;
        const double SHAPE_MIN_DIM_Z = 4.0;
        const double SHAPE_MAX_DIM_X = 7.0;
        const double SHAPE_MAX_DIM_Y = 7.0;
        const double SHAPE_MAX_DIM_Z = 7.0;
        const double SHAPE_THICKNESS = 0.5;
        const double SHAPE_PROBABILITY_L = 0.1;
        const double SHAPE_PROBABILITY_U = 0.05;
        const double SHAPE_PROBABILITY_RECTANGLE = 0.3;

        const char KW_SeparationObstacleObstacle[] = "SeparationObstacleObstacle";
        const char KW_SeparationObstacleInitial[] = "SeparationObstacleInitial";
        const char KW_SeparationObstacleGoal[] = "SeparationObstacleGoal";
        const char KW_SeparationInitialInitial[] = "SeparationInitialInitial";
        const char KW_SeparationInitialGoal[] = "SeparationInitialGoal";
        const char KW_SeparationGoalGoal[] = "SeparationGoalGoal";

        const double SEPARATION_OBSTACLE_OBSTACLE = 1.0;
        const double SEPARATION_OBSTACLE_INITIAL = 1.0;
        const double SEPARATION_OBSTACLE_GOAL = 0.5;
        const double SEPARATION_INITIAL_INITIAL = 1.0;
        const double SEPARATION_INITIAL_GOAL = 1.0;
        const double SEPARATION_GOAL_GOAL = 15;
    }
}

#endif
