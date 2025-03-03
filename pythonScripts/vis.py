from utils.figures import BLUE, GREEN, set_limits, color_isvalid, plot_coords
import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon
from descartes.patch import PolygonPatch
import pandas as pd
import numpy as np
import os
from shapely.geometry import LineString, Polygon, Point, MultiPolygon
from matplotlib.collections import LineCollection
import argparse

def find_position(x, y, points):
    for index, (point_x, point_y) in enumerate(points):
        if point_x == x and point_y == y:
            return index
    return None

def show(ii):
    root = "../data/path/"
    current_path = os.getcwd()

    print("当前工作目录是:", current_path)

    obstaclesID = str(ii)

    map_name = args.scene + "_" + str(args.grid)
    obstacle_file = "../data/Obstacles/" + map_name + "/" + obstaclesID + ".txt"
    predictLabel = "../data/PredictLabel/" + map_name + "/" + "predict_label_" + obstaclesID + ".txt"
    goalPosition = "../data/goalPosition/" + args.scene + "Points_" + str(args.grid ) + ".txt"

    x1, y1, x2, y2 =  17.5,4,32.5,46

    label = pd.read_csv(predictLabel, sep = ",", header = None, usecols= [0,1,2,3,4])
    label.columns = ['A', 'B', 'C', 'D', 'E']
    goalP = pd.read_csv(goalPosition, sep = ",", header = None)

    pStart = find_position(x1, y1, np.array(goalP))
    pGoal = find_position(x2, y2, np.array(goalP))

    p = label[pStart * 15 : pStart * 15 + 15]
    xx = pd.DataFrame({"A": [0], "B": [0], "C": [0], "D": [0], "E": [0]}, index=[pStart * 15 + pStart - 0.5])
    df = p.append(xx, ignore_index=False)
    df = df.sort_index().reset_index(drop=True)

    path = df.loc[pGoal]

    obstacles = []

    mm = pd.read_csv(obstacle_file)
    mm.columns = ['ha']

    for i in range(0, len(mm), 2):
        obstacle = []
        num = mm['ha'].iloc[i]
        poly = mm['ha'].iloc[i + 1].split(" ")
        for j in range(0, len(poly) - 1, 2):
            x = float(poly[j])
            y = float(poly[j + 1])
            obstacle.append((x, y))

        obstacles.append([obstacle, []])

    id = path[1]
    p_file = "../data/path/" + map_name + "/" + str(pStart) + "_" + str(pGoal) + "/" + str(id) + ".txt"
    dfs = pd.read_csv(p_file, header= None, sep=" ")
    dfs.columns = ['x', 'y']

    line_ = []
    for i in range(len(dfs)):
        line_.append((dfs['x'].iloc[i], dfs['y'].iloc[i]))

    linesss = LineString(line_).buffer(0.2, cap_style=1)

    x = len(obstacles)

    obstacles.extend([linesss])


    multi1 = MultiPolygon(obstacles)

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111)
    ax.set_facecolor('#FAFAD2')

    lines = []

    n = 0
    for polygon in multi1:
        if n >= x:
            exterior_coords = np.array(polygon.exterior.coords)
            lines.append(exterior_coords)
        else:
            plot_coords(ax, polygon.exterior, alpha=0)
            patch = PolygonPatch(polygon, facecolor=color_isvalid(multi1, valid=GREEN),
                                 edgecolor=color_isvalid(multi1, valid=GREEN), alpha=1, zorder=1)
            ax.add_patch(patch)

        n = n + 1

    lc = LineCollection(lines, colors='green')
    ax.add_collection(lc)

    # a = [42.5,
    #  7.5,
    # 43.700000000000003,
    # 7.0999999999999996,
    #  42.600000000000001,
    #  7.9000000000000004,
    #  41.799999999999997,
    #  8.5,
    #  41.5,
    #  9.5999999999999996,
    #  41.799999999999997,
    #  10.699999999999999,
    # 42.399999999999999,
    #  11.800000000000001,
    #  42.899999999999999,
    #  12.699999999999999,
    # 42.700000000000003,
    #  13.800000000000001,
    #  42.200000000000003,
    #  14.800000000000001,
    #  41.299999999999997,
    # 15.800000000000001,
    #  41.200000000000003,
    #  16.899999999999999,
    #  42,
    # 17.699999999999999,
    # ]
    #
    # for i in range(0, len(a), 2):
    #     plt.plot(a[i], a[i+1], 'b*')

    plt.plot(x1, y1, 'r*')
    plt.plot(x2, y2, 'b*')

    set_limits(ax, 0, 50, 0, 50)
    plt.rcParams['pdf.fonttype'] = 42
    plt.rcParams['ps.fonttype'] = 42

    plt.savefig("figures/" + str(ii) + ".jpg", bbox_inches = "tight")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Storage',
                        help="the scene of training data, 'Random' (default), Curves or Maze")
    parser.add_argument('--grid', type=int, default=4,
                        help="the number of goals")
    parser.add_argument('--fid', type=str, default=1,
                        help="the number of paths")
    parser.add_argument('-topNumber', type=int, default=5,
                        help="the number of paths")
    args = parser.parse_args()

    for i in range(0, 400):
        show(i)