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
#include "CreateScenes/CreateScene.hpp"
#include "External/PQP/PQPTriMesh.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Curve.hpp"
#include "Utils/Geometry.hpp"
#include "Utils/Logger.hpp"

namespace Antipatrea {

    Polygon2D *CreateScene::RandomShape(void) {
        const double coin = RandomUniformReal();

        if (coin < m_shapeProbL)
            return RandomShapeL();
        if (coin < (m_shapeProbL + m_shapeProbU))
            return RandomShapeU();
        if (coin < (m_shapeProbL + m_shapeProbU + m_shapeProbRect))
            return RandomShapeRectangle();

        return RandomShapeRegularPolygon();
    }

    Polygon2D *CreateScene::RandomShapeL(void) {
        Polygon2D *poly = new Polygon2D();
        const double dim1 = 2 * m_shapeThickness + RandomUniformReal(m_shapeMinDims[0], m_shapeMaxDims[0]);
        const double dim2 = 2 * m_shapeThickness + RandomUniformReal(m_shapeMinDims[0], m_shapeMaxDims[0]);
        std::vector<double> skel;
        std::vector<double> vertices;

        skel.push_back(-dim1);
        skel.push_back(0.0);
        skel.push_back(0.0);
        skel.push_back(0.0);
        skel.push_back(0.0);
        skel.push_back(dim2);

        FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], m_shapeThickness, vertices);
        poly->AddVertices(vertices.size() / 2, &vertices[0]);

        return poly;
    }

    Polygon2D *CreateScene::RandomShapeU(void) {
        Polygon2D *poly = new Polygon2D();
        const double dim1 = 2 * m_shapeThickness + RandomUniformReal(m_shapeMinDims[0], m_shapeMaxDims[0]);
        const double dim2 = 2 * m_shapeThickness + RandomUniformReal(m_shapeMinDims[0], m_shapeMaxDims[0]);
        const double dim3 = 2 * m_shapeThickness + RandomUniformReal(m_shapeMinDims[0], m_shapeMaxDims[0]);
        std::vector<double> skel;
        std::vector<double> vertices;

        skel.push_back(-0.5 * dim1);
        skel.push_back(dim2);
        skel.push_back(-0.5 * dim1);
        skel.push_back(0.0);
        skel.push_back(0.5 * dim1);
        skel.push_back(0.0);
        skel.push_back(0.5 * dim1);
        skel.push_back(dim3);

        FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], m_shapeThickness, vertices);
        poly->AddVertices(vertices.size() / 2, &vertices[0]);

        return poly;
    }

    Polygon2D *CreateScene::RandomShapeRectangle(void) {
        Polygon2D *poly = new Polygon2D();
        const double xdim = RandomUniformReal(m_shapeMinDims[0], m_shapeMaxDims[0]);
        const double ydim = RandomUniformReal(m_shapeMinDims[1], m_shapeMaxDims[1]);
        const double min[] = {-0.5 * xdim, -0.5 * ydim};
        const double max[] = {0.5 * xdim, 0.5 * ydim};
        double vertices[8];

        BoxAsPolygon2D(min, max, vertices);
        poly->AddVertices(4, vertices);

        return poly;
    }

    Polygon2D *CreateScene::RandomShapeRegularPolygon(void) {
        const int nv = RandomUniformInteger(m_shapeMinNrVertices, m_shapeMaxNrVertices);
        const double r = RandomUniformReal(m_shapeMinDims[0], m_shapeMaxDims[0]);
        Polygon2D *poly = new Polygon2D();
        std::vector<double> vertices;

        vertices.resize(2 * nv);
        CircleAsPolygon2D(0, 0, r, nv, &vertices[0]);
        poly->AddVertices(vertices.size() / 2, &vertices[0]);

        return poly;
    }

    void CreateScene::RandomPlacement(Polygon2D &poly, const double r1,  const double r2, const double c[], const double x2, const double y2)
    {
        const double *min = m_grid->GetMin();
        const double *max = m_grid->GetMax();
        const double offx = 0.0; // 0.1 * (max[0] - min[0]);
        const double offy = 0.0; // 0.1 * (max[1] - min[1]);

        //double TA[3] = {RandomUniformReal(min[0] + offx, max[0] - offx), RandomUniformReal(min[1] + offy, max[1] - offy), RandomUniformReal(m_shapeMinAngle, m_shapeMaxAngle)};
        double TA[3];
        double TR[Algebra2D::TransRot_NR_ENTRIES];
        // if(r1 > 0 && r2 > 0 && r1 <= r2 &&  c != NULL)
        // {
        //     do
        //   SampleRandomPointInsideAnnulus2D(c, r1,  r2,  TA);
        //     while(TA[0] < (min[0] + offx) || TA[0] > (max[0]  - offx)  || TA[1] < (min[1] + offy)  || TA[1] > (max[1] - offy));
        // }

        TA[0] = x2 + RandomUniformReal(-2, 2);
        TA[1] = y2 + RandomUniformReal(-2, 2);
        TA[2] = RandomUniformReal(m_shapeMinAngle, m_shapeMaxAngle);

        //Logger::m_out << " init = " << c[0] << " " << c[1] << " goal = " << TA[0] << " " << TA[1] << " d = " << Algebra2D::PointDistance(c, TA) << std::endl;

        Algebra2D::TransAngleAsTransRot(TA, TR);
        poly.ApplyTransRot(TR);
    }


    bool CreateScene::IsAcceptable(const int gid, TriMesh &tmesh) {
        for (int i = m_groups.size() - 1; i >= 0; --i) {
            const double dsep = GetSeparation(gid, i);
            if (dsep > 0.0 && tmesh.Distance(NULL, NULL, &(m_groups[i]->m_tmesh), NULL, NULL) < dsep)
                return false;
        }
        return true;
    }

    bool CreateScene::AddObject(const int gid, const int nrTries, const double r1,  const double r2, const double c[], const double x2, const double y2)
    {
        if(m_grid->GetNrDims() == 2)
            return AddPolygon2D(gid, nrTries, r1, r2, c, x2, y2);
        else
            return AddBox3D(gid, nrTries, r1, r2, c, x2, y2);

    }

    bool CreateScene::AddPolygon2D(const int gid, const int nrTries, const double r1, const double r2, const double c[], const double x2, const double y2)
    {
        for (int i = 0; i < nrTries; ++i)
        {
            //Logger::m_out << "nrTries" << i << std::endl;
            Polygon2D *poly = RandomShape();
            RandomPlacement(*poly, r1,  r2, c, x2, y2);
            if (IsAcceptable(gid, *poly))
            {
                m_groups[gid]->m_objects.push_back(poly);
                m_groups[gid]->m_tmesh.AddPolygon(*poly);
                return true;
            }
            delete poly;
        }
        return false;
    }


    bool CreateScene::AddBox3D(const int gid, const int nrTries, const double r1,  const double r2, const double center[], const double x2, const double y2)
    {
        double box[6];
        double c;
        double d;


        for(int i = 0; i < nrTries; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                c = RandomUniformReal(m_grid->GetMin()[j], m_grid->GetMax()[j]);
                d = RandomUniformReal(m_shapeMinDims[j], m_shapeMaxDims[j]);
                box[j] = c - 0.5 * d;
                box[j + 3] = c + 0.5 * d;
            }
            if(IsAcceptable(gid, box))
            {
                m_groups[gid]->m_tmesh.AddBox(box[0], box[1], box[2], box[3], box[4], box[5]);
                for(int j = 0; j < 6; ++j)
                    m_groups[gid]->m_boxes3D.push_back(box[j]);
                return true;

            }
        }
        return false;


    }

}
