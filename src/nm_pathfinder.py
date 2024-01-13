import queue

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

def path_to(source_point, source_box, destination_box, came_from, detail_points):
    path_points = [source_point]
    current = destination_box
    path_boxes = []
    while current != source_box:
        destination_point = legal_path_between(source_point, source_box, destination_box, detail_points)
        path_points.append(destination_point)
        source_point = destination_point
        path_boxes.append(current)
        source_box = current
        destination_box = came_from[current]
    path_boxes.append(source_box)

    # While node.prev != null: keep going
    # prev_node = destination.prev
    # path = []
    # while prev_node != None:
    #     path.append(prev_node)
    #     prev_node = prev_node.prev
    return path_points 

def breadth_first_search(source_point, source_box, destination_box, adjacencies, visited_boxes, detail_points):
    came_from = dict()
    queue = [source_box]
    while queue:
        current_node = queue.pop()
        visited_boxes.append(current_node)
        if current_node == destination_box:
            return path_to(source_point, source_box, destination_box, came_from, detail_points)
        else:
            for new_node in adjacencies[current_node]:
                if came_from.get(new_node) == None:
                    came_from[new_node] = current_node
                    queue.append(new_node)

    return False

def legal_path_between(source_point, source_box, destination_box, detail_points):
    destination_point = []
    # x/y range of border is the acceptable range that we can move to from our current point.
    x_range_of_border = [max(source_box[0], destination_box[0]), min(source_box[1], source_box[1])]
    y_range_of_border = [max(source_box[2], destination_box[2]), min(source_box[3], source_box[3])]
    # diff from x/y is the absolute distance between the original point's X/Y and the two x values of the max/min of the box.
    # We're doing these calculations only for if the x/y point is outside the min/max bounds of the other box's respective trimmed x/y range.
    diff_from_x = [abs(x_range_of_border[0]-source_point[0]), abs(x_range_of_border[1]-source_point[0])]
    diff_from_y = [abs(y_range_of_border[0]-source_point[1]), abs(y_range_of_border[1]-source_point[1])]
    # if the distance between the point and x1 is smaller than the distance from x2, 
    # the destination point should be x1 (because we want the values that are the closet to the point) 
    destination_point.append(diff_from_x[0] if diff_from_x[0] < diff_from_x[1] else diff_from_x[1])
    destination_point.append(diff_from_y[0] if diff_from_y[0] < diff_from_y[1] else diff_from_y[1])
    # match the source box with the bounds of the destination box
    detail_points[source_box] = destination_point
    return destination_point

def find_path (source_point, destination_point, mesh):
    # Mesh: has a list of boxes and a list of adjacencies

    # We are searching through the entire dictionary of boxes to find the boxes with the src/desination points in them.
    
    src_dst_boxes = []
    visited_boxes = []
    detail_points = {}
    
    for current_box in mesh["boxes"]:
        if (source_point == destination_point):
            if is_point_in_box(source_point, current_box):
                return ([source_point, destination_point], current_box)
        if is_point_in_box(source_point, current_box) or is_point_in_box(destination_point, current_box):
            print("box of current point: ", current_box)
            src_dst_boxes.append(current_box)
    # print("mesh list:", mesh["adj"])
    print("visited boxes: ", src_dst_boxes, " length: ", len(src_dst_boxes), " type: ", type(src_dst_boxes))

    if (len(src_dst_boxes) == 2):
        print("source box: ", src_dst_boxes[0], " dest box: ", src_dst_boxes[1])
        # print(legal_path_between(source_point, src_dst_boxes[0], src_dst_boxes[1]))

        result = breadth_first_search(source_point, src_dst_boxes[0], src_dst_boxes[1], mesh["adj"], visited_boxes, detail_points)
        if (result == False):
            print("No path!")
            return
        else:
           return result, visited_boxes
            
    else: 
        print("No path!")
        return
    # print(visited_boxes)

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
