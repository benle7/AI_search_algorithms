# Ben Levi 318811304


"""
    The utils.py file implements all the methods that are relevant
    to the search algorithms and the exercise requirements.
"""


import random
import csv
import os
from ways import load_map_from_csv
from ways import compute_distance
from ways import draw
from ways.info import SPEED_RANGES
import heapq as pq
import math
import time


try:
    import matplotlib.pyplot as plt
except ImportError:
    raise ImportError('Please install matplotlib:  http://matplotlib.org/users/installing.html#windows')

plt.axis('equal')

# Max length of path for create_problems function.
MAX_LENGTH = 20
# Max speed of all the links types.
MAX_SPEED = 110
# Global variable for 'ida*' algorithm.
new_limit = 0


# ***** The function restores the path after one of the algorithms has found it. *****
def get_path(close, target):
    current = target
    result = []
    # close is dict between key-node to value-parent.
    # It store the path from the source to target,
    # and close[source] = source (stop condition).
    while current != close[current]:
        result.append(current)
        current = close[current]
    result.append(current)
    result.reverse()
    return result


def huristic_func(lat1, lon1, lat2, lon2):
    return compute_distance(lat1, lon1, lat2, lon2) / MAX_SPEED


def cost_func(lnk):
    # Divide by 1000, for convert m to km (because speed in km/h).
    return (lnk.distance / 1000) / SPEED_RANGES[lnk.highway_type][1]


def dfs_f(current, g_val, path, f_limit, target, junctions, h):
    global new_limit
    new_f = g_val + h(junctions[current].lat, junctions[current].lon, junctions[target].lat, junctions[target].lon)
    if new_f > f_limit:
        new_limit = min(new_limit, new_f)
        return []
    if current == target:
        return path
    for lnk in junctions[current].links:
        sol = dfs_f(lnk.target, g_val + cost_func(lnk),
                    path + [lnk.target], f_limit, target, junctions, h)
        if sol:
            return sol
    return []


def idastar_func(source, target, junctions, h):
    global new_limit
    # 0 + h (because g(source) is 0).
    new_limit = h(junctions[source].lat, junctions[source].lon, junctions[target].lat, junctions[target].lon)
    while True:
        f_limit = new_limit
        new_limit = math.inf
        sol = dfs_f(source, 0, [source], f_limit, target, junctions, h)
        if sol:
            return sol


def call_idastar(source, target, h_func):
    junctions = load_map_from_csv().junctions()
    return idastar_func(source, target, junctions, h_func)


def astar_func(source, target, junctions, h):
    target_lat = junctions[target].lat
    target_lon = junctions[target].lon
    open_lst = []
    pq.heapify(open_lst)
    # 0 + h (because g(source) is 0).
    # heap (by the first value in the tuple) of tuples : (g+h, g, index, parent_index).
    pq.heappush(open_lst, (h(junctions[source].lat, junctions[source].lon, target_lat, target_lon),
                           0, source, source))
    # close is dict between key-node to value-parent.
    close = dict({})

    while len(open_lst) > 0:
        next_node = pq.heappop(open_lst)
        # after pop from the heap -> save the parent of the node in close dict.
        close[next_node[2]] = next_node[3]
        if next_node[2] == target:
            return get_path(close, target)
        next_links = junctions[next_node[2]].links
        for lnk in next_links:
            j = lnk.target
            if j not in close:
                new_g = next_node[1] + cost_func(lnk)
                old = [(h_val, g_old, node_index, parent) for h_val, g_old, node_index, parent
                       in open_lst if node_index == j]
                if old:
                    if old[0][1] > new_g:
                        open_lst.remove(old[0])
                        # h + new_g  (old[0][0] - old[0][1] = old_g + h - old_g = h)
                        open_lst.append((new_g + old[0][0] - old[0][1], new_g, j, next_node[2]))
                        pq.heapify(open_lst)
                else:
                    pq.heappush(open_lst, (new_g + h(junctions[j].lat, junctions[j].lon, target_lat, target_lon),
                                           new_g, j, next_node[2]))
    return None


def call_astar(source, target, h_func):
    junctions = load_map_from_csv().junctions()
    return astar_func(source, target, junctions, h_func)


def ucs_func(source, target, junctions):
    open_lst = []
    pq.heapify(open_lst)
    # g(source) is 0.
    # heap (by the first value in the tuple) of tuples : (g, index, parent_index).
    pq.heappush(open_lst, (0, source, source))
    # close is dict between key-node to value-parent.
    close = dict({})

    while len(open_lst) > 0:
        next_node = pq.heappop(open_lst)
        # after pop from the heap -> save the parent of the node in close dict.
        close[next_node[1]] = next_node[2]
        if next_node[1] == target:
            return get_path(close, target)
        next_links = junctions[next_node[1]].links
        for lnk in next_links:
            j = lnk.target
            if j not in close:
                new_g = next_node[0] + cost_func(lnk)
                old = [(g_old, node_index, parent) for g_old, node_index, parent in open_lst if node_index == j]
                if old:
                    if old[0][0] > new_g:
                        open_lst.remove(old[0])
                        open_lst.append((new_g, j, next_node[1]))
                        pq.heapify(open_lst)
                else:
                    pq.heappush(open_lst, (new_g, j, next_node[1]))
    return None


def call_ucs(source, target):
    junctions = load_map_from_csv().junctions()
    return ucs_func(source, target, junctions)


# ***** Function for SECTION-12 *****
def send_idastar_to_draw(h_func):
    junctions = load_map_from_csv().junctions()
    default_problems = [(81777, 81731), (287504, 287506), (694347, 694353), (888889, 888913), (145001, 145018),
                        (190525, 190522), (944770, 944768), (36053, 36056), (83733, 83741), (71225, 71211)]
    result = []
    for source, target in default_problems:
        result.append(idastar_func(source, target, junctions, h_func))

    dir_name = "solutions_img"
    if not os.path.isdir(dir_name):
        os.mkdir(dir_name)
    count = 1
    for path in result:
        draw.plot_path(junctions, path)
        plt.savefig(os.path.join(dir_name, "img" + str(count) + ".png"))
        # *** need plt.show() ? ***
        plt.show()
        count = count + 1


# ***** Function for SECTION-9.2 *****
def astar_times_for_graph(h_func):
    filename1 = "problems.csv"
    if not os.path.exists(filename1):
        create_problems()
    junctions = load_map_from_csv().junctions()
    result = []
    with open(filename1, 'r', newline='') as csv_file:
        csv_reader = csv.reader(csv_file)
        for source, target in csv_reader:
            result.append(astar_func(int(source), int(target), junctions, h_func))

    x_lst = []
    y_lst = []
    for path in result:
        current_time = 0
        h_result = h_func(junctions[path[0]].lat, junctions[path[0]].lon,
                          junctions[path[-1]].lat, junctions[path[-1]].lon)
        for s, t in zip(path, path[1:]):
            lnk = [link for link in junctions[s].links if link.target == t][0]
            current_time += cost_func(lnk)

        x_lst.append(h_result)
        y_lst.append(current_time)

    plt.plot(x_lst, y_lst, 'o')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('graph')
    plt.show()


# ***** Function for SECTION-9.1 *****
def send_problems_astar(h_func):
    junctions = load_map_from_csv().junctions()
    result = []
    filename1 = "problems.csv"
    with open(filename1, 'r', newline='') as csv_file:
        csv_reader = csv.reader(csv_file)
        for source, target in csv_reader:
            result.append(astar_func(int(source), int(target), junctions, h_func))

    for path in result:
        current_time = 0
        h_result = h_func(junctions[path[0]].lat, junctions[path[0]].lon,
                          junctions[path[-1]].lat, junctions[path[-1]].lon)
        for s, t in zip(path, path[1:]):
            lnk = [link for link in junctions[s].links if link.target == t][0]
            current_time += cost_func(lnk)

        path.append('-')
        path.append(round(current_time, 4))
        path.append('-')
        path.append(round(h_result, 4))

    filename2 = "results/AStarRuns.txt"
    if os.path.exists(filename2):
        os.remove(filename2)
    with open(filename2, 'w+') as fp:
        for path in result:
            fp.write(' '.join([str(x) for x in path]))
            fp.write('\n')


# ***** Function for SECTION-5 *****
def send_problems_ucs():
    junctions = load_map_from_csv().junctions()
    result = []
    filename1 = "problems.csv"
    with open(filename1, 'r', newline='') as csv_file:
        csv_reader = csv.reader(csv_file)
        for source, target in csv_reader:
            result.append(ucs_func(int(source), int(target), junctions))

    for path in result:
        current_time = 0
        for s, t in zip(path, path[1:]):
            lnk = [link for link in junctions[s].links if link.target == t][0]
            current_time += cost_func(lnk)

        path.append('-')
        path.append(round(current_time, 4))

    filename2 = "results/UCSRuns.txt"
    if os.path.exists(filename2):
        os.remove(filename2)
    with open(filename2, 'w+') as fp:
        for path in result:
            fp.write(' '.join([str(x) for x in path]))
            fp.write('\n')


# ***** Function for SECTION-3 *****
def create_problems():
    junctions = load_map_from_csv().junctions()
    filename = "problems.csv"
    problems = []
    for i in range(1, 101):
        source = random.choice(junctions)
        target = source
        current = 0
        while current < MAX_LENGTH and len(target.links) > 0:
            target = junctions[random.choice(target.links).target]
            current += 1

        problems.append([source.index, target.index])

    if os.path.exists(filename):
        os.remove(filename)
    with open(filename, 'w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerows(problems)


# ***** Function for SECTION-13 *****
def take_times(h_func):
    junctions = load_map_from_csv().junctions()

    default_problems = [(81777, 81731), (287504, 287506), (694347, 694353), (888889, 888913), (145001, 145018),
                        (190525, 190522), (944770, 944768), (36053, 36056), (83733, 83741), (71225, 71211)]
    n = len(default_problems)

    total = 0
    for source, target in default_problems:
        start = time.time()
        ucs_func(source, target, junctions)
        end = time.time()
        total = total + (end - start)
    print("UCS average time : " + str(total / n))

    total = 0
    for source, target in default_problems:
        start = time.time()
        astar_func(source, target, junctions, h_func)
        end = time.time()
        total = total + (end - start)
    print("A* average time : " + str(total / n))

    total = 0
    for source, target in default_problems:
        start = time.time()
        idastar_func(source, target, junctions, h_func)
        end = time.time()
        total = total + (end - start)
    print("IDA* average time : " + str(total / n))
