from math import sqrt
from shapely import affinity

GM = (sqrt(5)-1.0)/2.0
W = 8.0
H = W*GM
SIZE = (W, H)

BLUE = '#6699cc'
GRAY = '#999999'
DARKGRAY = '#333333'
YELLOW = '#ffcc33'
GREEN = '#339933'
RED = '#ff3333'
BLACK = '#000000'
DARKVIOLET = '#9400D3'
SIENNA = '#A0522D'

COLOR_ISVALID = {
    True: BLACK,
    False: BLACK,
}

def plot_line(ax, ob, color=BLACK, zorder=10, linewidth=1, alpha=1):
    x, y = ob.xy
    ax.plot(x, y, color=color, linewidth=linewidth , solid_capstyle='round', zorder=zorder, alpha=alpha)

def plot_coords(ax, ob, color=BLACK, zorder=1, alpha=0.1, type = None, order = None, distance = None):
    x, y = ob.xy
    for i in range(len(x)):
        ax.plot(x[i], y[i], 'o', color=color, zorder=zorder, alpha=alpha)
        if type == "line":
            if i == len(x) - 1:
                ax.annotate(text=str(i) + "_(" + str(order[i]) + ")_",
                            xy=(x[i], y[i]), xytext=(x[i], y[i] + 1))
            else:
                ax.annotate(text=str(i) + "_(" + str(order[i]) + ")" + str(int(distance[order[i]][order[i + 1]])),
                            xy=(x[i], y[i]), xytext=(x[i], y[i] + 1))
            #ax.annotate(text=str(i), xy=(x[i], y[i]), xytext=(x[i], y[i] + 1))

def color_isvalid(ob, valid=BLUE, invalid=BLUE):
    if ob.is_valid:
        return valid
    else:
        return invalid

def color_issimple(ob, simple=BLUE, complex=BLUE):
    if ob.is_simple:
        return simple
    else:
        return complex

def plot_line_isvalid(ax, ob, **kwargs):
    kwargs["color"] = color_isvalid(ob)
    plot_line(ax, ob, **kwargs)

def plot_line_issimple(ax, ob, **kwargs):
    kwargs["color"] = color_issimple(ob)
    plot_line(ax, ob, **kwargs)

def plot_bounds(ax, ob, zorder=1, alpha=1):
    x, y = zip(*list((p.x, p.y) for p in ob.boundary))
    ax.plot(x[0], y[0], 'o', color='#0c23f2', zorder=zorder, alpha=alpha)
    ax.plot(x[1], y[1], 'o', color='#00ff2a', zorder=zorder, alpha=alpha)

def add_origin(ax, geom, origin):
    x, y = xy = affinity.interpret_origin(geom, origin, 2)
    ax.plot(x, y, 'o', color=BLACK, zorder=1)
    ax.annotate(str(xy), xy=xy, ha='center',
                textcoords='offset points', xytext=(0, 8))

def set_limits(ax, x0, xN, y0, yN):
    ax.set_xlim(x0, xN)
    ax.set_xticks(range(x0, xN+1, 10))
    ax.set_ylim(y0, yN)
    ax.set_yticks(range(y0, yN+1, 10))
    ax.set_aspect("equal")