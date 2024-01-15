import queue
import math
from heapq import heappop, heappush


def is_point_in_box(point, box):
    # box: [left x corner, right x corner, top y corner, bottom y corner]
    # point: [x, y]
    x1 = box[0]
    x2 = box[1]
    y1 = box[2]
    y2 = box[3]
    
    if (point[0] > x1 and point[0] < x2 and point[1] > y1 and point[1] < y2 ):
        return True
    return False

def path_to(destination_point, source_box, destination_box, came_from, detail_points):
    # We should only be here if we found a path the canonical FINAL destination box.
    # The source point we're passing in here is the canonical source point.
    path_points = [destination_point]
    current_box = destination_box
    # path_boxes = []
    # We are working backwards here, in destination -> source, but also to current box -> came_from[current_box].
    while current_box != source_box:
        source_point = legal_path_between(destination_point, came_from[current_box], current_box, detail_points)
        path_points.append(source_point)
        destination_point = source_point
        current_box = came_from[current_box]
    return path_points 

def breadth_first_search(destination_point, source_box, destination_box, adjacencies, visited_boxes, detail_points):
    came_from = dict()
    q = queue.Queue()
    q.put(source_box)
    came_from[source_box] = None
    while q.qsize() != 0:
        current_box = q.get()
        visited_boxes.append(current_box)
        if current_box == destination_box:
            return path_to(destination_point, source_box, destination_box, came_from, detail_points)
        else:
            for new_box in adjacencies[current_box]:
                if came_from.get(new_box) == None:
                    came_from[new_box] = current_box
                    q.put(new_box)

    return False


def get_euclidean_distance(p1: tuple, p2:  tuple):
    x = 0
    y = 1

    distance = math.pow((p2[x] - p1[x]), 2) + pow((p2[y] - p1[y]), 2)
    normalized_distance = math.sqrt(distance)
    return normalized_distance


# Some things to remember:
# f cost: total cost of the node. h + g.
# g cost: distance between the current node and the start node.
# h cost: estimated cost to destination. JUST AN ESTIMATE.
# find: f = h + g
def a_star_search(destination_point, source_box, destination_box, adjacencies, visited_boxes, detail_points):
    came_from = dict()
    g_costs = dict()
    h_costs = dict()
    f_costs = dict()

    q = []
    heappush(q, (0, source_box)) 
    g_costs[source_box] = 0
    # Calculate h cost for source box 
    # Also calculate f cost, or total cost
    h_costs[source_box] = get_euclidean_distance(source_box, destination_box)
    f_costs[source_box] = g_costs[source_box] + h_costs[source_box] # f = g + h
    came_from[source_box] = None

    while q:
        f_cost, current_box = heappop(q)
        visited_boxes.append(current_box)
        if current_box == destination_box:
            return path_to(destination_point, source_box, destination_box, came_from, detail_points)
        else:
            for new_box in adjacencies[current_box]:
                new_g_cost = g_costs[current_box] + get_euclidean_distance(new_box, current_box)
                new_h_cost = get_euclidean_distance(new_box, destination_box)
                new_f_cost = new_h_cost + new_g_cost

                if came_from.get(new_box) == None or new_f_cost < f_costs[new_box]:
                    # if the new total cost is less than the current cost to get to that node
                    came_from[new_box] = current_box
                    g_costs[new_box] = new_g_cost
                    h_costs[new_box] = new_h_cost
                    f_costs[new_box] = new_f_cost
                    heappush(q, (new_f_cost, new_box))
                
    return False


def legal_path_between(source_point, source_box, destination_box, detail_points):
    destination_point = ()

    p_x = source_point[0]
    p_y = source_point[1]

    x1 = 0
    x2 = 1
    
    y1 = 0
    y2 = 1

    box_x1 = 0
    box_x2 = 1
    box_y1 = 2
    box_y2 = 3

    # x/y range of border is the acceptable range that we can move to from our current point.
    x_range_of_border = [max(source_box[box_x1], destination_box[box_x1]), min(source_box[box_x2], source_box[box_x2])]
    y_range_of_border = [max(source_box[box_y1], destination_box[box_y1]), min(source_box[box_y2], source_box[box_y2])]
    ## x 
    # if x < min
    if (p_x < x_range_of_border[x1]):
        destination_point += (x_range_of_border[x1],) # min val of range
    # if x > max
    elif (p_x > x_range_of_border[x2]):
        destination_point += (x_range_of_border[x2],) # max val of range
    # if min < x < max
    else:
        destination_point += (p_x,)
    ## y
    # if y < min
    if (p_y < y_range_of_border[y1]):
        destination_point += (y_range_of_border[y1], ) # min val of range
    # if y > max
    elif (p_y > y_range_of_border[y2]):
        destination_point += (y_range_of_border[y2],) # max val of range
    # if min < y < max
    else:
        destination_point += (p_y, )
    # match the source box with the bounds of the destination box
    detail_points[source_box] = destination_point
    return destination_point

def get_euclidean_distance(p1: tuple, p2:  tuple):
    x = 0
    y = 1

    distance = math.pow((p2[x] - p1[x]), 2) + pow((p2[y] - p1[y]), 2)
    normalized_distance = math.sqrt(distance)
    return normalized_distance




def find_path (source_point, destination_point, mesh):
    # Mesh: has a list of boxes and a list of adjacencies

    # We are searching through the entire dictionary of boxes to find the boxes with the src/desination points in them.
    source_box = None
    destination_box = None
    # src_dst_boxes = []
    visited_boxes = []
    detail_points = {}
    
    for current_box in mesh["boxes"]:
        if (source_point == destination_point):
            if is_point_in_box(source_point, current_box):
                return ([source_point, destination_point], current_box)
        if is_point_in_box(source_point, current_box):
            source_box = current_box
        elif  is_point_in_box(destination_point, current_box):
            destination_box = current_box
            

    if (source_box != None and destination_box != None):
        result = a_star_search(destination_point, source_box, destination_box, mesh["adj"], visited_boxes, detail_points)
        if (result == False):
            print("No path!")
            return [], visited_boxes
        else:
           result.append(source_point)
           result.reverse()
           return result, visited_boxes
            
    else: 
        print("No path!")
        return [], visited_boxes

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    # path = []
    # boxes = {}

    # return path, boxes.keys()
