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
#include "Components/Scene2D.hpp"
#include "Utils/Geometry.hpp"
#include <fstream>

using namespace Antipatrea;


extern "C" void CreateSceneCurves(int argc, char **argv) {
    Scene2D scene;
    const double thick = CreateScene::DefaultThicknessScene2D();

    int n;


//obstacles
    std::vector<double> skel;
    std::vector<double> vertices;
    Polygon2D *poly;

    for (double x = -40; x <= 15; x += 2) {
        skel.push_back(x);
        skel.push_back(-29 + sin(0.1 * x) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);

    skel.clear();
    for (double x = 40; x >= -23; x -= 2) {
        skel.push_back(x);
        skel.push_back(-20.2 + cos(0.2 * x) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);

    skel.clear();
    for (double x = -40; x <= 15; x += 2) {
        skel.push_back(x);
        skel.push_back(-10 + sin(0.2 * (fabs(x))) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);


    skel.clear();
    for (double x = 36; x >= 20; x -= 2) {
        skel.push_back(x);
        skel.push_back(-10 + sin(0.2 * (fabs(x))) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);


    skel.clear();
    for (double x = 40; x >= -5; x -= 2) {
        skel.push_back(x);
        skel.push_back(2 + sin(0.2 * x) * sin(0.1 * x) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);


    skel.clear();
    for (double x = -30; x <= 5; x += 2) {
        skel.push_back(x);
        skel.push_back(10 - (x * x) / (130) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);


    skel.clear();
    for (double x = -40; x <= 20; x += 2) {
        skel.push_back(x);
        skel.push_back(15 + x / 40 * 3 * sin(0.2 * x));
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);


    skel.clear();
    for (double x = 40; x >= -20; x -= 2) {
        skel.push_back(x);
        skel.push_back(29 - (x * x) / (130) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);


    skel.clear();
    for (double x = -40; x <= -15; x += 2) {
        skel.push_back(x);
        skel.push_back(32 + sin(0.2 * x) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);


    skel.clear();
    for (double x = 40; x >= 15; x -= 2) {
        skel.push_back(x);
        skel.push_back(32 + sin(0.2 * x) * 2);
    }
    poly = new Polygon2D();
    FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
    poly->AddVertices(vertices.size() / 2, &vertices[0]);
    scene.AddObstacle(poly);

    std::ofstream out("data/SceneCurvesObstacles.txt", std::ofstream::out);
    scene.PrintObstacles(out);
    out.close();


}


