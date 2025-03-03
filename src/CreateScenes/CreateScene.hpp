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

#ifndef Antipatrea__CreateScene_HPP_
#define Antipatrea__CreateScene_HPP_

#include "CreateScenes/Constants.hpp"
#include "Utils/Misc.hpp"
#include "Utils/Params.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/TriMeshDefault.hpp"
#include <ostream>

namespace Antipatrea {

    class CreateScene {
    public:
        CreateScene(void)
                : m_shapeMinNrVertices(Constants::SHAPE_MIN_NR_VERTICES),
                  m_shapeMaxNrVertices(Constants::SHAPE_MAX_NR_VERTICES), m_shapeThickness(Constants::SHAPE_THICKNESS),
                  m_shapeProbL(Constants::SHAPE_PROBABILITY_L), m_shapeProbU(Constants::SHAPE_PROBABILITY_U),
                  m_shapeProbRect(Constants::SHAPE_PROBABILITY_RECTANGLE) {
            m_shapeMinDims[0] = Constants::SHAPE_MIN_DIM_X;
            m_shapeMinDims[1] = Constants::SHAPE_MIN_DIM_Y;
            m_shapeMinDims[2] = Constants::SHAPE_MIN_DIM_Z;
            m_shapeMaxDims[0] = Constants::SHAPE_MAX_DIM_X;
            m_shapeMaxDims[1] = Constants::SHAPE_MAX_DIM_Y;
            m_shapeMaxDims[2] = Constants::SHAPE_MAX_DIM_Z;

            m_shapeMinAngle = -M_PI;
            m_shapeMaxAngle = M_PI;

        }

        virtual ~CreateScene(void) {
            DeleteItems<Group *>(m_groups);
        }

        struct Group {
            virtual ~Group(void) {
                DeleteItems<Polygon2D *>(m_objects);
            }

            std::vector<Polygon2D *> m_objects;
            std::vector<double> m_boxes3D;
            TriMeshDefault m_tmesh;
        };

        static double DefaultDimScene2D(void) {
            return 80.0;
        }

        static double DefaultThicknessScene2D(void) {
            return 1.0;
        }


        virtual const std::vector<Group *> *GetGroups(void) const {
            return &m_groups;
        }

        virtual std::vector<Group *> *GetGroups(void) {
            return &m_groups;
        }

        virtual double GetSeparation(const int i, const int j) const {
            return m_dseps[i * m_groups.size() + j];
        }

        virtual const Grid *GetGrid(void) const {
            return m_grid;
        }

        virtual void SetupFromParams(Params &p) {
            m_shapeMinNrVertices = p.GetValueAsInt(Constants::KW_ShapeMinNrVertices, m_shapeMinNrVertices);
            m_shapeMaxNrVertices = p.GetValueAsInt(Constants::KW_ShapeMaxNrVertices, m_shapeMaxNrVertices);

            p.GetValuesAsDoubles(Constants::KW_ShapeMinDims, m_shapeMinDims, 3);
            p.GetValuesAsDoubles(Constants::KW_ShapeMaxDims, m_shapeMaxDims, 3);

            m_shapeThickness = p.GetValueAsDouble(Constants::KW_ShapeThickness, m_shapeThickness);

            m_shapeProbL = p.GetValueAsDouble(Constants::KW_ShapeProbabilityL, m_shapeProbL);
            m_shapeProbU = p.GetValueAsDouble(Constants::KW_ShapeProbabilityU, m_shapeProbU);
            m_shapeProbRect = p.GetValueAsDouble(Constants::KW_ShapeProbabilityRectangle, m_shapeProbRect);
        }

        virtual void SetSeparation(const int i, const int j, const double d) {
            const int n = m_groups.size();
            if (m_dseps.size() < n * n)
                m_dseps.resize(n * n);
            m_dseps[i * n + j] = m_dseps[j * n + i] = d;
        }

        virtual void SetGrid(const Grid *const grid) {
            m_grid = grid;
        }


        virtual bool
        AddObject(const int gid, const int nrTries, const double r1 = -1, const double r2 = -1, const double c[] = NULL,
                  const double x2 = -1, const double y2 = -1);

        virtual bool AddPolygon2D(const int gid, const int nrTries, const double r1 = -1, const double r2 = -1,
                                  const double c[] = NULL, const double x2 = -1, const double y2 = -1);

        virtual bool
        AddBox3D(const int gid, const int nrTries, const double r1 = -1, const double r2 = -1, const double c[] = NULL,
                 const double x2 = -1, const double y2 = -1);

        virtual Polygon2D *RandomShape(void);

        virtual Polygon2D *RandomShapeL(void);

        virtual Polygon2D *RandomShapeU(void);

        virtual Polygon2D *RandomShapeRectangle(void);

        virtual Polygon2D *RandomShapeRegularPolygon(void);

        virtual void
        RandomPlacement(Polygon2D &poly, const double r1 = -1, const double r2 = -1, const double c[] = NULL,
                        const double x2 = -1, const double y2 = -1);

        virtual bool IsAcceptable(const int gid, TriMesh &tmesh);

        virtual bool IsAcceptable(const int gid, Polygon2D &poly) {
            TriMeshDefault tmesh;
            tmesh.AddPolygon(poly);
            return IsAcceptable(gid, tmesh);
        }


        virtual bool IsAcceptable(const int gid, const double box[]) {
            TriMeshDefault tmesh;
            tmesh.AddBox(box[0], box[1], box[2], box[3], box[4], box[5]);
            return IsAcceptable(gid, tmesh);
        }


//protected:
        const Grid *m_grid;
        std::vector<double> m_dseps;
        std::vector<Group *> m_groups;
        double m_shapeMinNrVertices;
        double m_shapeMaxNrVertices;
        double m_shapeMinDims[3];
        double m_shapeMaxDims[3];
        double m_shapeThickness;
        double m_shapeProbL;
        double m_shapeProbU;
        double m_shapeProbRect;
        double m_shapeMinAngle;
        double m_shapeMaxAngle;


    };
}

#endif
