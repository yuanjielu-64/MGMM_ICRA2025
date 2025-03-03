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
#include "Components/Scene2D.hpp"
#include "Components/Constants.hpp"
#include "External/ShewchukTriangle.hpp"
#include "Utils/GDraw.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

namespace Antipatrea {
    Scene2D::Scene2D(void) {
        m_grid.Setup(2, Constants::GRID_DIMS, Constants::GRID_MIN, Constants::GRID_MAX);
        m_heightObstacle = Constants::SCENE_OBSTACLE_HEIGHT;
    }

    void Scene2D::SetupFromParams(Params &params) {
        std::vector<Polygon2D *> polys;

        m_heightObstacle = params.GetValueAsDouble(Constants::KW_ObstacleHeight, m_heightObstacle);

        auto name = params.GetValue(Constants::KW_ObstaclesPolygonsFile);
        auto id = params.GetValue(Constants::KW_ObstaclesPolygonsID);
        auto map = params.GetValue(Constants::KW_UseSceneForGrid);
        auto grid = params.GetValue(Constants::KW_UseGrid);
        auto mgmmResults = params.GetValue(Constants::KW_MGMMPredictLabel);
        auto topNrPaths = params.GetValueAsInt(Constants::KW_TopNumber);;

        auto goalPosition = "data/goalPosition/" + std::string(map) + "Points_" + std::string(grid) + ".txt";

        auto filePath = std::string(name) + std::string(map) + "_" + std::string(grid) + "/" + std::string(id) + ".txt";
        auto predictPath = std::string(mgmmResults) + std::string(map) + "_" + std::string(grid) + +"/predict_label_" +
                           std::string(id) + ".txt";

        Logger::m_out << Constants::KW_ObstaclesPolygonsFile << " <" << filePath << ">" << std::endl;
        Logger::m_out << "the predict result is from " << " <" << predictPath << ">" << std::endl;

        std::fstream new_file;
        new_file.open(predictPath, std::ios::in);

        if (new_file.is_open()) {
            //checking whether the file is open
            std::string line;
            for (int i = 0; i <= std::stod(grid) * std::stod(grid) - 1; ++i) {
                for (int j = 0; j <= std::stod(grid) * std::stod(grid) - 1; ++j) {
                    if (i != j) {
                        std::getline(new_file, line);
                        std::stringstream ss(line);
                        std::string word;
                        int n = 0;
                        double dis = 0.0;

                        std::vector<std::vector<double>> path;
                        while (std::getline(ss, word, ',')) {
                            if (n < topNrPaths) {
                                std::string pathID = "data/path/" + std::string(map) + "_" + std::string(grid) + "/" + std::to_string(i) + "_" + std::to_string(j) + "/" + std::string(word) + ".txt";

                                std::fstream new_path;
                                new_path.open(pathID, std::ios::in);

                                if (new_path.is_open()){
                                    std::string line1;
                                    std::vector<double> xy;
                                    while(std::getline(new_path, line1)){
                                        std::stringstream sss(line1);
                                        std::string word1;

                                        while (std::getline(sss, word1, ' '))
                                            xy.push_back(stod(word1));

                                    }

                                    path.push_back(xy);
                                    new_path.close();
                                }
                            } else
                                dis += stod(word);

                            n++;
                        }

                        m_mmPredictedPaths.push_back(path);
                        m_mmMatrix.push_back(dis / topNrPaths);

                    }
                    else{
                        std::vector<std::vector<double>> path;
                        m_mmPredictedPaths.push_back(path);
                        m_mmMatrix.push_back(0);
                    }
                }
            }

            new_file.close(); //close the file object.
        }

        std::fstream new_file1;
        new_file1.open(goalPosition, std::ios::in);

        if (new_file1.is_open()) {
            std::string line;
            std::vector<std::vector<double>> cors;
            while(std::getline(new_file1, line)){
                std::stringstream ss(line);
                std::string word;
                std::vector<double> cor;
                while (std::getline(ss, word, ','))
                    cor.push_back(stod(word));

                m_mmCors.push_back(cor);
            }
            new_file1.close();
        }

        params.SetValue("predictPath", predictPath.c_str());

        if (filePath.c_str()) {
            ReadPolygons2D(filePath.c_str(), polys);
            for (auto &poly: polys)
                AddObstacle(poly);
            Logger::m_out << "read " << polys.size() << " polygons" << std::endl;
        }

        Scene::SetupFromParams(params);

        m_tmeshTerrain.AddBox2D(m_grid.GetMin()[0],
                                m_grid.GetMin()[1],
                                m_grid.GetMax()[0],
                                m_grid.GetMax()[1]);

    }

    void Scene2D::AddObstacle(Polygon2D *const poly) {
        m_obstacles.push_back(poly);
        m_tmeshObstaclesCollision.AddPolygon(*poly);
        m_tmeshObstaclesDraw.AddExtrudedPolygon(*poly, 0, m_heightObstacle);
    }

    void Scene2D::AddBoundaries(const double thick, const double h) {
        m_tmeshObstaclesCollision.AddBoundaries2D(m_grid.GetMin()[0], m_grid.GetMin()[1], m_grid.GetMax()[0],
                                                  m_grid.GetMax()[1], thick);
        m_tmeshObstaclesDraw.AddBoundaries(m_grid.GetMin()[0], m_grid.GetMin()[1], 0.0, m_grid.GetMax()[0],
                                           m_grid.GetMax()[1], h, thick);
    }

    void
    Scene2D::Triangulate(std::vector<double> &triVertices, std::vector<int> &triIndices, std::vector<int> &triNeighs,
                         const double triAvgArea) {
        std::vector<double> vertices;
        std::vector<int> nrVerticesPerContour;
        std::vector<double> ptsInsideHoles;
        const int nrObsts = GetNrObstacles();
        Polygon2D *poly;

        const double *pmin = GetGrid()->GetMin();
        const double *pmax = GetGrid()->GetMax();

        // grid boundaries
        vertices.push_back(pmin[0]);
        vertices.push_back(pmin[1]);
        vertices.push_back(pmax[0]);
        vertices.push_back(pmin[1]);
        vertices.push_back(pmax[0]);
        vertices.push_back(pmax[1]);
        vertices.push_back(pmin[0]);
        vertices.push_back(pmax[1]);
        nrVerticesPerContour.push_back(4);

        // obstacles as holes
        ptsInsideHoles.resize(2 * nrObsts);
        for (int i = 0; i < nrObsts; ++i) {
            poly = GetObstacle(i);
            nrVerticesPerContour.push_back(poly->GetNrVertices());
            poly->GetRepresentativePoint(&ptsInsideHoles[2 * i]);

            vertices.insert(vertices.end(), poly->GetVertices()->begin(), poly->GetVertices()->end());
        }

        Logger::m_out << "calling triangulation" << std::endl;

        TriangulatePolygonWithHoles2D(false, -1, triAvgArea, vertices.size() / 2, &vertices[0],
                                      &nrVerticesPerContour[0], ptsInsideHoles.size() / 2,
                                      &ptsInsideHoles[0], &triVertices, &triIndices, &triNeighs);

        Logger::m_out << "done" << std::endl;
    }


    void Scene2D::DrawObstaclesPolygons(void) {
        for (int i = m_obstacles.size() - 1; i >= 0; --i)
            GDrawPolygon2D(*(m_obstacles[i]));
    }

    void Scene2D::SampleValidBoxCenter(const double dims[], double c[]) {
        TriMeshDefault tmesh;
        auto x = 0.5 * dims[0];
        auto y = 0.5 * dims[1];
        const double box[] = {-x, -y, x, y};

        do {
            for (int j = m_grid.GetNrDims() - 1; j >= 0; --j)
                c[j] = RandomUniformReal(m_grid.GetMin()[j], m_grid.GetMax()[j]);
            tmesh.Clear();
            tmesh.AddBox2D(box[0] + c[0], box[1] + c[1], box[2] + c[0], box[3] + c[1]);
        } while (m_tmeshObstaclesCollision.Collision(NULL, NULL, &tmesh, NULL, NULL) == true);
    }


}
