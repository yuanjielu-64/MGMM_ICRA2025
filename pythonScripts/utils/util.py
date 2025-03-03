from shapely.geometry import LineString, Polygon, Point, MultiPolygon
import matplotlib.pyplot as plt
from .figures import *
from descartes.patch import PolygonPatch
import random
import numpy as np
from matplotlib.collections import LineCollection
#random.seed(1)

def cut_circle(obstacles, times, path):
    total = []
    for circle in obstacles:
        new_obstacles = []
        xy = []
        for i in range(len(circle.coords.xy[0]) - 1):
            xy.append((circle.coords.xy[0][i], circle.coords.xy[1][i]))

        l1 = random.randint(0, len(circle.coords.xy[0]))
        for j in range(l1):
            ll = xy.pop(0)
            xy.append(ll)

        new_obstacles.append(xy)
        # cut
        time = 0
        while(True):
            if len(new_obstacles[0]) <= 6:
                if len(new_obstacles[0]) >= 2:
                    line = LineString(new_obstacles[0]).buffer(0.2, cap_style=2)
                    if abs(line.distance(path)) >= 0.3:
                        total.append(line)

                a = new_obstacles.pop(0)

            else:
                x = new_obstacles.pop(0)
                l2 = random.randint(0, len(x))
                if l2 - 1 >= 2:
                    new_obstacles.append(x[: l2 - 1])
                if l2 + 1 <= len(x):
                    new_obstacles.append(x[l2: ])
                time += 1

            if time > times:
                break

        for i in range(len(new_obstacles)):
            if len(new_obstacles[i]) >= 4:
                line = LineString(new_obstacles[i]).buffer(0.2, cap_style=2)
                if abs(line.distance(path)) >= 0.3:
                    total.append(line)

    return total

def generate_map(figureType, line, lens, distance, min_distance, obstacles, type, is_angle = False, close_path = False, dfs = None, flag = False):
    count = 0
    error = 0
    start = 1.0

    #print("processing")
    while (True):
        xy = []
        if flag == True:
            x = np.random.triangular(np.min(dfs.x) - 5, np.min(dfs.x) + (np.max(dfs.x) - np.min(dfs.x)) / 2, np.max(dfs.x) + 5, 1)
            y = np.random.triangular(np.min(dfs.y) - 5, np.min(dfs.y) + (np.max(dfs.y) - np.min(dfs.y)) / 2, np.max(dfs.y) + 5, 1)
        elif random.uniform(0, 1) < 0.5:
            x = np.random.triangular(0, 25, 50, 1)
            y = np.random.triangular(0, 25, 50, 1)
        else:
            x = np.random.uniform(0, 50)
            y = np.random.uniform(0, 50)
        buffer = np.random.uniform(2, 4)
        rotate = np.random.uniform(-180, 180)

        for i in range(len(line)):
            xy.append((line[i][0] * start + x, line[i][1] *  start + y))

        if type == "line":
            xx = 0
            if is_angle == True:
                rotate = random.choice([-180, -90, 90, 180])
            node = affinity.rotate(LineString(xy).buffer(0.2, cap_style=1), rotate, 'center')
        elif type == "polygon":
            xx = 1
            node = affinity.rotate(Polygon(xy), rotate, 'center')
        else:
            xx = 1
            simplyif = random.uniform(0, 1)
            node = affinity.rotate(Point(xy).buffer(buffer).simplify(simplyif, preserve_topology=True), rotate, 'center')

        if check_is_valid(node, obstacles, distance, close_path):
            figureType.append(xx)
            obstacles.append(node)
            count += 1
        else:
            error += 1

        if count >= lens:
            break

        if error % 2 == 0 and error != 0 and flag != True:
            start = 0.98 * start

        if error % 2 == 0 and error != 0 and np.random.uniform(0, 1) <= 0.8:
            distance = distance - 0.1

        if distance <= min_distance:
            break

    del xy

    return obstacles

def check_is_valid(node, obstacles, distance, close_path = False):
    # check boundary
    for i in range(len(node.bounds)):
        if node.bounds[i] > 50 or node.bounds[i] < 0:
            return False

    # check it is overlap
    for i in range(len(obstacles)):
        if i == 0:
            if close_path == True:
                if obstacles[i].exterior.distance(node) <= 1:
                    return False
                elif obstacles[i].exterior.distance(node) >= 2:
                    return False
                elif obstacles[i].contains(node) == True:
                    return False
            else:
                if obstacles[i].exterior.distance(node) <= 1:
                    return False

                elif obstacles[i].contains(node) == True:
                    return False
        else:
            if obstacles[i].exterior.distance(node) <= distance:
                return False

            elif obstacles[i].contains(node) == True:
                return False

    return True

def visualize_map(obstacles, name, n, x, y, figureType):
    # figureType 0 -> line, 1 -> polygon

    multi1 = MultiPolygon(obstacles)
    px = 1 / plt.rcParams['figure.dpi']  # pixel in inches
    my_dpi = 96
    fig = plt.figure(figsize=(100 * 100/(my_dpi * 38), 100 * 100/(my_dpi * 38)), dpi=my_dpi)
    ax = fig.add_subplot(111)
    #ax.set_facecolor('#FAFAD2')

    # quick build
    lines = []
    n = 0
    for polygon in obstacles:
        if figureType[n] == 0:
            exterior_coords = np.array(polygon.exterior.coords)
            lines.append(exterior_coords)

            for interior in polygon.interiors:
                interior_coords = np.array(interior.coords)
                lines.append(interior_coords)

        n += 1
    lc = LineCollection(lines, colors='k')
    ax.add_collection(lc)

    n = 0
    for polygon in obstacles:
        if figureType[n] == 1:
            plot_coords(ax, polygon.exterior, alpha=0)
            patch = PolygonPatch(polygon, facecolor='black', edgecolor='black', alpha=1, zorder=1)
            ax.add_patch(patch)
        n += 1

    ax.set_axis_off()
    ax.set_frame_on(False)
    set_limits(ax, 0, 50, 0, 50)

    plt.rcParams['path.simplify'] = True
    plt.rcParams['path.simplify_threshold'] = 1.0

    #plt.savefig(name + ".jpg", dpi=my_dpi, bbox_inches='tight', pad_inches=0)
    #plt.close("all")

    plt.show()

def visualize_path(obstacles, name, n, x, y, figureType):
    # figureType 0 -> line, 1 -> polygon

    multi1 = MultiPolygon(obstacles)
    px = 1 / plt.rcParams['figure.dpi']  # pixel in inches
    my_dpi = 96
    fig = plt.figure(figsize=(100 * 100/(my_dpi * 38), 100 * 100/(my_dpi * 38)), dpi=my_dpi)
    ax = fig.add_subplot(111)
    #ax.set_facecolor('#FAFAD2')

    # quick build
    lines = []
    n = 0
    for polygon in obstacles:
        if figureType[n] == 0:
            exterior_coords = np.array(polygon.exterior.coords)
            lines.append(exterior_coords)

            for interior in polygon.interiors:
                interior_coords = np.array(interior.coords)
                lines.append(interior_coords)

        n += 1
    lc = LineCollection(lines, colors='k')
    ax.add_collection(lc)

    n = 0
    for polygon in obstacles:
        if figureType[n] == 1:
            plot_coords(ax, polygon.exterior, alpha=0)
            patch = PolygonPatch(polygon, facecolor='black', edgecolor='black', alpha=1, zorder=1)
            ax.add_patch(patch)
        n += 1

    ax.set_axis_off()
    ax.set_frame_on(False)
    set_limits(ax, 0, 50, 0, 50)

    plt.rcParams['path.simplify'] = True
    plt.rcParams['path.simplify_threshold'] = 1.0

    #plt.savefig(name + ".jpg", dpi=my_dpi, bbox_inches='tight', pad_inches=0)
    #plt.close("all")

    plt.show()

def save_txt(obstacles, lens, name):
    path = name

    with open(path, 'w') as f:
        f.write(str(lens) + '\n')
        for i in range(len(obstacles)):
            node = obstacles[i].exterior.coords.xy
            f.write(str(len(node[0]) - 1) + '\n')
            for j in range(len(node[0]) - 1):
                a = round(node[0][j], 5)
                b = round(node[1][j], 5)
                if a <= 0:
                    a = 0
                elif a >= 50:
                    a = 50
                if b <= 0:
                    b = 0
                elif b >= 50:
                    b = 50
                f.write(str(round(a, 5)) + " " + str(round(b, 5)) + " ")
            f.write('\n')
            f.write('\n')