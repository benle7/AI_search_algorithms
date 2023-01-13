# Ben Levi 318811304


"""
Parse input and run appropriate code.
Don't use this file for the actual work; only minimal code should be here.
We just parse input and call methods from other modules.
"""


# do NOT import ways. This should be done from other files
# simply import your modules and call the appropriate functions
import utils


def huristic_function(lat1, lon1, lat2, lon2):
    return utils.huristic_func(lat1, lon1, lat2, lon2)


def find_ucs_rout(source, target):
    """call function to find path, and return list of indices"""
    return utils.call_ucs(source, target)


def find_astar_route(source, target):
    """call function to find path, and return list of indices"""
    return utils.call_astar(source, target, huristic_function)


def find_idastar_route(source, target):
    """call function to find path, and return list of indices"""
    return utils.call_idastar(source, target, huristic_function)
    

def dispatch(argv):
    from sys import argv
    source, target = int(argv[2]), int(argv[3])
    if argv[1] == 'ucs':
        path = find_ucs_rout(source, target)
    elif argv[1] == 'astar':
        path = find_astar_route(source, target)
    elif argv[1] == 'idastar':
        path = find_idastar_route(source, target)
    print(' '.join(str(j) for j in path))


if __name__ == '__main__':
    from sys import argv

    """ ***** SECTION-3 ***** """
    # utils.create_problems()

    """ ***** SECTION-5 ***** """
    # utils.send_problems_ucs()

    """ ***** SECTION-9.1 ***** """
    # utils.send_problems_astar(huristic_function)

    """ ***** SECTION-9.2 ***** """
    # utils.astar_times_for_graph(huristic_function)

    """ ***** SECTION-12 ***** """
    # utils.send_idastar_to_draw(huristic_function)

    """ ***** SECTION-13 ***** """
    # utils.take_times(huristic_function)

    dispatch(argv)
