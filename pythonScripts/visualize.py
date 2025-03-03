from utils.figures import BLUE, set_limits, color_isvalid, plot_coords
import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon
from descartes.patch import PolygonPatch
import pandas as pd
import numpy as np
import os
from shapely.geometry import LineString, Polygon, Point, MultiPolygon
from matplotlib.collections import LineCollection

root = "../data/path/"
current_path = os.getcwd()

print("当前工作目录是:", current_path)

map_name = "Curves_3"
for sg_name in os.listdir(root + map_name + "/"):
        for path in os.listdir(root + map_name + "/" + sg_name + "/"):
            path_file = root + map_name + "/" + sg_name + "/" + path
            obstacle_file = "../data/obstacles/" + map_name  + "/" + sg_name + "/" + path

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

            multi1 = MultiPolygon(obstacles)

            dfs = pd.read_csv(path_file, sep=" ", header=None)
            dfs.columns = ['x', 'y']
            line_ = []
            for i in range(len(dfs)):
                line_.append((dfs['x'].iloc[i], dfs['y'].iloc[i]))

            paths = LineString(line_).buffer(0.5, cap_style=1)

            obstacles.extend([paths])

            fig = plt.figure(figsize=(6, 6))
            ax = fig.add_subplot(111)
            ax.set_facecolor('#FAFAD2')

            lines = [np.array(paths.exterior.coords)]

            for polygon in multi1:
                exterior_coords = np.array(polygon.exterior.coords)
                lines.append(exterior_coords)

            lc = LineCollection(lines, colors='k')
            ax.add_collection(lc)


            set_limits(ax, 0, 50, 0, 50)
            plt.rcParams['pdf.fonttype'] = 42
            plt.rcParams['ps.fonttype'] = 42
            if os.path.exists("../data/orifigures/" + map_name) == False:
                os.makedirs("../data/orifigures/" + map_name)

            if os.path.exists("../data/orifigures/" + map_name + "/" + sg_name)  == False:
                os.makedirs("../data/orifigures/" + map_name + "/" + sg_name)

            plt.savefig("../data/orifigures/" + map_name + "/" + sg_name + "/" + path[:-4] + ".jpg", bbox_inches="tight")
            #plt.show()