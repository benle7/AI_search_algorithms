# Ben Levi 318811304


"""
This file should be runnable to print map_statistics using 
$ python stats.py
"""


from collections import namedtuple
from ways import load_map_from_csv
from collections import Counter


def map_statistics(roads):
    """
    return a dictionary containing the desired information
    You can edit this function as you wish
    """
    Stat = namedtuple('Stat', ['max', 'min', 'avg'])
    max_deg = None
    min_deg = None
    max_dis = None
    min_dis = None
    avg_deg = None
    avg_dis = None
    histogram = None

    junctions = roads.junctions()
    amount_junctions = len(junctions)
    amount_links = 0
    sum_dis = 0
    list_types = []
    for j in junctions:
        amount_links += len(j.links)
        for lnk in j.links:
            sum_dis += lnk.distance
            list_types.append(lnk.highway_type)

    if amount_junctions > 0:
        filter_junctions = list(filter(lambda jun: len(jun.links) > 0, junctions))
        max_deg = len(max(junctions, key=lambda jun: len(jun.links)).links)
        min_deg = len(min(junctions, key=lambda jun: len(jun.links)).links)
        avg_deg = amount_links / amount_junctions
        if amount_links > 0:
            max_dis_junction = max(filter_junctions, key=lambda jun: max(jun.links, key=lambda l: l.distance).distance)
            max_dis = max(max_dis_junction.links, key=lambda l: l.distance).distance
            min_dis_junction = min(filter_junctions, key=lambda jun: min(jun.links, key=lambda l: l.distance).distance)
            min_dis = min(min_dis_junction.links, key=lambda l: l.distance).distance
            avg_dis = sum_dis / amount_links
            histogram = dict(Counter(list_types))

    return {
        'Number of junctions': amount_junctions,
        'Number of links': amount_links,
        'Outgoing branching factor': Stat(max=max_deg, min=min_deg, avg=avg_deg),
        'Link distance': Stat(max=max_dis, min=min_dis, avg=avg_dis),
        # value should be a dictionary
        # mapping each road_info.TYPE to the no' of links of this type
        'Link type histogram': histogram,  # tip: use collections.Counter
    }


def print_stats():
    for k, v in map_statistics(load_map_from_csv()).items():
        print('{}: {}'.format(k, v))

        
if __name__ == '__main__':
    from sys import argv
    assert len(argv) == 1
    print_stats()

