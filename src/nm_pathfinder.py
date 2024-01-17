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

def path_to(source_point, destination_point, source_box, destination_box, forward_path, backward_path, middle_box, middle_point, detail_points):
    # find middle point
    visited_boxes = []
    # From middle point -> source
    forward_path_points = [middle_point]
    current_box = middle_box
    # as long as the current box have parents, find the path between the current box and its parents
    dp = middle_point
    while current_box != source_box and current_box != destination_box and forward_path[current_box] != None:
        sp = legal_path_between(dp, forward_path[current_box], current_box, detail_points)
        forward_path_points.append(sp)
        visited_boxes.append(current_box)
        dp = sp
        current_box = forward_path[current_box]
    forward_path_points.append(source_point)
    visited_boxes.append(source_box)

    # same thing but from middle box to destination
    backward_path_points = []
    current_box = middle_box
    dp = middle_point
    while current_box != destination_box and current_box != source_box and backward_path[current_box] != None:
        sp = legal_path_between(dp, backward_path[current_box], current_box, detail_points)
        backward_path_points.append(sp)
        visited_boxes.append(current_box)
        dp = sp
        current_box = backward_path[current_box]
    backward_path_points.append(destination_point)
    visited_boxes.append(destination_box)
    # connect the entire path
    forward_to_mid = forward_path_points[::-1]
    forward_to_mid.extend(backward_path_points)
    return forward_to_mid, visited_boxes

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

    distance = math.pow((p2[x] - p1[x]), 2) + math.pow((p2[y] - p1[y]), 2)
    normalized_distance = math.sqrt(distance)
    return normalized_distance

# bidirectional
def bidirectional_a_star_search(source_point, destination_point, source_box, destination_box, adjacencies, visited_boxes, detail_points):
    forward_path = dict()
    forward_costs = dict()
    backward_path = dict()
    backward_costs = dict()
    forward_closed = list()
    backward_closed = list()
    q = []
    # tuple of (g cost, h cost, f cost)
    # Calculate h cost for source box and f cost, or total cost
    heappush(q, (0, source_box, destination_box, source_point))  # adding the forward search
    forward_h_cost = get_euclidean_distance(source_point, destination_point)
    forward_costs[source_box] = (0, forward_h_cost, forward_h_cost) 
    forward_path[source_box] = None
    # same for backward
    heappush(q, (0, destination_box, source_box, destination_point))  # adding the backward search
    backward_h_cost = get_euclidean_distance(destination_point, source_point)
    backward_costs[destination_box] = (0, backward_h_cost, backward_h_cost) 
    backward_path[destination_box] = None
    # While we have elements in the queue
    while q:
        # pop an element
        cost, current_box, current_goal, entry_point = heappop(q)
        visited_boxes.append(current_box)
        # Check which path it's on based on its goal
        if (current_goal == destination_box):
            forward_closed.append(current_box)
            # If the current box is also on the other path, that means it's the middle point
            # calculate the entire path
            # For all of its neighbors, calculate the g,h,f costs
            for new_box in adjacencies[current_box]:
                new_entry = legal_path_between(entry_point, current_box, new_box, detail_points)
                new_g_cost = forward_costs[current_box][0] + get_euclidean_distance(new_entry, entry_point)
                new_h_cost = get_euclidean_distance(new_entry, destination_point)
                new_f_cost = new_h_cost + new_g_cost
                # if box is new OR the new calculated cost is less than it's current cost
                if (new_box not in forward_path or new_f_cost < forward_costs[new_box][2]):
                    # update the queue
                    if forward_path[current_box] is not new_box and new_box not in forward_closed: 
                        forward_path[new_box] = current_box
                        forward_costs[new_box] = (new_g_cost, new_h_cost, new_f_cost)
                        heappush(q, (forward_costs[new_box][2], new_box, current_goal, new_entry))
            if (current_box in backward_closed):
                return path_to(source_point, destination_point, source_box, destination_box, forward_path, backward_path, current_box, entry_point, detail_points)

        else:
            backward_closed.append(current_box)
            for new_box in adjacencies[current_box]:
                new_entry = legal_path_between(entry_point, current_box, new_box, detail_points)
                new_g_cost = backward_costs[current_box][0] + get_euclidean_distance(new_entry, entry_point)
                new_h_cost = get_euclidean_distance(new_entry, source_point)
                new_f_cost = new_h_cost + new_g_cost
                if (new_box not in backward_path or new_f_cost < backward_costs[new_box][2]):
                    if backward_path[current_box] is not new_box and new_box not in backward_closed:
                        backward_path[new_box] = current_box
                        backward_costs[new_box] = (new_g_cost, new_h_cost, new_f_cost)
                        heappush(q, (backward_costs[new_box][2], new_box, current_goal, new_entry))
            # exit
            if (current_box in forward_closed):
                return path_to(source_point, destination_point, source_box, destination_box, forward_path, backward_path, current_box, entry_point, detail_points)
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
    x_range_of_border = [max(source_box[box_x1], destination_box[box_x1]), min(source_box[box_x2], destination_box[box_x2])]
    y_range_of_border = [max(source_box[box_y1], destination_box[box_y1]), min(source_box[box_y2], destination_box[box_y2])]
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
    # assert (x_range_of_border[x1] == x_range_of_border[x2] and (destination_point[1] == y_range_of_border[y1] or destination_point[1] == y_range_of_border[y2])) or (y_range_of_border[y1] == y_range_of_border[y2] and (destination_point[0] == x_range_of_border[x1] or destination_point[0] == x_range_of_border[x2]))
    
    return destination_point

# Used in BFS, A*, etc., but not in the bidirectional
def old_path_to(destination_point, source_box, destination_box, came_from, detail_points):
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
            return old_path_to(destination_point, source_box, destination_box, came_from, detail_points)
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

def find_path (source_point, destination_point, mesh):
    # Mesh: has a list of boxes and a list of adjacencies
    # We are searching through the entire dictionary of boxes to find the boxes with the src/desination points in them.
    source_box = None
    destination_box = None
    # src_dst_boxes = []
    visited_boxes = []
    detail_points = {}

    for current_box in mesh["boxes"]:
        if is_point_in_box(source_point, current_box):
            source_box = current_box
        if is_point_in_box(destination_point, current_box):
            destination_box = current_box

    if (source_box != None and destination_box != None):
        if (destination_box in mesh["adj"][source_box] or source_box in mesh["adj"][destination_box]):
            mid_point = legal_path_between(source_point, destination_box, source_box, detail_points)
            return ([source_point, mid_point, destination_point], [source_box, destination_box])
        if (destination_box == source_box):
            mid_point = legal_path_between(source_point, destination_box, source_box, detail_points)
            return ([source_point, mid_point, destination_point], [source_box])

        result = bidirectional_a_star_search(source_point, destination_point, source_box, destination_box, mesh["adj"], visited_boxes, detail_points)
        # result = a_star_search(destination_point, source_box, destination_box, mesh["adj"], visited_boxes, detail_points)

        if (result == False):
            print("No path!")
            return [], visited_boxes
        else:
        #    result.append(source_point)
        #    result.reverse()
           return result[0], visited_boxes
            
    else: 
        print("Please click on the white areas of the picture, we couldn't find your source/destination points!")
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
