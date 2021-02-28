#!/usr/bin/env python
# followed tutorial https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/ for the SimpleActionClient from T1

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
import actionlib
import rospy
import tf
from sklearn.cluster import AgglomerativeClustering
import numpy as np
import random

if __name__ == '__main__':
    rospy.init_node('project2_go_to_goal', anonymous=True)

    listener = tf.TransformListener()

    marker_publisher = rospy.Publisher('frontier_markers', MarkerArray)

    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    sac.wait_for_server()

    # occupancy grid from event
    last_grid = None
    # array; [[points in frontier]...]
    # indices match unique_labels
    frontiers = None
    # array; [centroid...]
    # indices match unique_labels
    centroids = None
    # array; [label...]
    unique_labels = None

    # checks if any adjacent positions are empty in an occupancy grid
    def is_empty_nearby(data, x, y, width, height):
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue
                # check bounds
                if x + dx >= 0 and x + dx < width and y + dy >= 0 and y + dy < height:
                    if data[(y + dy) * width + (x + dx)] == 0:
                        return True
        return False

    # https://stackoverflow.com/questions/24852345/hsv-to-rgb-color-conversion
    # convert hsv [1,1,1] to rgb [1,1,1]
    def hsv_to_rgb(h, s, v):
        if s == 0.0: return (v, v, v)
        i = int(h*6.) # assume int() truncates!
        f = (h*6.)-i; p,q,t = v*(1.-s), v*(1.-s*f), v*(1.-s*(1.-f)); i%=6
        if i == 0: return (v, t, p)
        if i == 1: return (q, v, p)
        if i == 2: return (p, v, t)
        if i == 3: return (p, q, v)
        if i == 4: return (t, p, v)
        if i == 5: return (v, p, q)

    # create a marker with preset headers and the given values
    def make_marker(id, position, hsv, scale, mode):
        marker = Marker()
        marker.ns = "frontier_markers"
        marker.header.frame_id = '/map'
        marker.type= marker.SPHERE
        marker.action = mode
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 1
        marker.lifetime = rospy.Time(0)
        marker.color.a = 1
        
        rgb = hsv_to_rgb(hsv[0], hsv[1], hsv[2])

        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.pose.orientation.w = 1

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.id = id

        return marker

    #last_marker_ids = None

    # sends a request to remove all old markers
    def delete_old_markers():
        marker_array = MarkerArray()
        marker_array.markers = [make_marker(1, (0,0,0), (0,0,0), 1, 3)]
        marker_publisher.publish(marker_array)

    # converts an occupancy grid position to world position
    def grid_to_world(grid_pos, grid_info):
        x = grid_pos[0]
        y = grid_pos[1]

        x = grid_info.origin.position.x + x * grid_info.resolution
        y = grid_info.origin.position.y + y * grid_info.resolution
        return (x, y, grid_info.origin.position.z)

    # calculates centroid as shown in the class slides
    def calculate_centroid(frontier_world_positions):
        x_c = 0.0
        y_c = 0.0
        count = float(len(frontier_world_positions))

        for world_pos in frontier_world_positions:
            x_c = x_c + world_pos[0]
            y_c = y_c + world_pos[1]
        
        return (x_c / count, y_c / count)

    # grows squares of a certain radius around known space
    def grow_obstacles(data, width, height, radius):
        # copy the list so we dont interfere with ourselves
        data_temp = data[:]
        # iterate over all values of x and y
        for x in range(0, width):
            for y in range(0, height):
                # check if occupied
                if data[x + y * width] == 100:
                    # iterate over all offsets dx and dy in square
                    for dx in range(-1 * radius, radius + 1):
                        for dy in range(-1 * radius, radius + 1):
                            # check for 0 offset
                            if dx != 0 or dy != 0:
                                # grow obstacle
                                data_temp[x + dx + ((y + dy) * width)] = 100
        return data_temp

    def form_frontiers_and_centroids(positions, labels, grid_info):
        positions = np.array(positions)
        labels = np.array(labels)

        unique_labels = list(np.unique(labels))
        frontiers = [positions[labels == label] for label in unique_labels]
        centroids = [calculate_centroid([grid_to_world(position, grid_info) for position in frontier]) for frontier in frontiers]
        return (frontiers, centroids, unique_labels)

    # creates all markers for a given iteration
    def publish_markers(frontiers, centroids, unique_labels, grid_info):
        global last_marker_ids
        delete_old_markers()

        # used for creating equidistant HSV colors
        deg_interval = 1.0 / len(unique_labels)

        #rospy.loginfo("deg_interval {}".format(deg_interval))

        marker_array = MarkerArray()

        hsv_colors = [(deg_interval * i, 1, 1) for i in range(len(unique_labels))]

        #rospy.loginfo("colors {}".format(hsv_colors))

        # build markers for frontier points
        # marker_array.markers = [make_marker(i, grid_to_world(positions[i], grid_info), hsv_colors[unique_labels.index(labels[i])], 0.05, 0) for i in range(len(labels))]
        marker_array.markers = []
        for i, frontier in enumerate(frontiers):
            marker_array.markers += [make_marker(random.randint(0, 2147000000), grid_to_world(position, grid_info), hsv_colors[i], 0.05, 0) for position in list(frontier)]

        # append centroid markers to original markers
        marker_array.markers = marker_array.markers + [make_marker(random.randint(0, 2147000000), centroid, hsv_colors[i], 0.25, 0) for i, centroid in enumerate(centroids)]
        
        # extract marker ids for reference
        #marker_ids = [marker.id for marker in marker_array.markers]
        #last_marker_ids = marker_ids

        # publish markers
        marker_publisher.publish(marker_array)
        rospy.loginfo("published {} markers".format(len(marker_array.markers)))

    def recalc_frontiers():
        if last_grid == None:
            return
        
        data = last_grid

        width = data.info.width
        height = data.info.height
        # functions to convert a given 1d grid position to a 2d coordinate pair part
        y = lambda i: i // data.info.width
        x = lambda i: i % data.info.width

        # obstacle growth
        new_occ_grid = grow_obstacles(list(data.data), width, height, 4)

        # convert occupancy grid into frontier map occupancy grid
        new_occ_grid = [100 if is_empty_nearby(new_occ_grid, x(i), y(i), width, height) and new_occ_grid[i] == -1 else 0 for i in range(0, len(new_occ_grid))]

        occupancy_grid = new_occ_grid
        data2 = data
        data2.data = new_occ_grid
        frontiers_publisher.publish(data2)

        np_occ = np.array(occupancy_grid)

        # zip elements with their positions
        ## I saw your error you commented in Canvas. That is normally due to mismatched axes but I do not encounter it in my code. 
        ## Perhaps we have different versions of numpy. Hopefully this fixes it: the subarray is now a tuple so there should be no question
        ## of the shape of this array.
        np_occ = np.array(zip(np_occ, [(x(i), y(i)) for i in range(0, len(occupancy_grid))]))
        # create a boolean mask for elements that are frontier elements
        np_occ_mapping = np.apply_along_axis(lambda x: x[0] == 100, 1, np_occ)
        # apply mask
        np_occ_filtered = np_occ[np_occ_mapping]

        if len(np_occ_filtered) == 0:
            # no more explorable frontiers
            global frontiers
            global centroids
            global unique_labels
            frontiers = None
            centroids = None
            unique_labels = None
            delete_old_markers()
            rospy.loginfo("No more explorable frontiers!")
            return
        
        del np_occ_mapping
        # map array to only the position
        np_occ = np.apply_along_axis(lambda x: x[1], 1, np_occ_filtered)
        del np_occ_filtered
        
        #rospy.loginfo(np_occ.shape)
        #rospy.loginfo("width: {}, height: {}".format(data.info.width, data.info.height))

        clustering = AgglomerativeClustering(linkage="ward", n_clusters=min(4, len(np_occ)))
        labels = clustering.fit_predict(np_occ)

        #rospy.loginfo(labels)

        global frontiers
        global centroids
        global unique_labels
        frontiers, centroids, unique_labels = form_frontiers_and_centroids(np_occ, labels, data.info)
        
        publish_markers(frontiers, centroids, unique_labels, data.info)


    # callback for occupancy grid subscriber
    def on_occupancy_grid(data):
        global last_grid
        last_grid = data

    rospy.Subscriber("/map", OccupancyGrid, on_occupancy_grid)
    frontiers_publisher = rospy.Publisher("/frontiers_map", OccupancyGrid)

    # find index of next largest frontier
    def get_next_frontier_index():
        larglen = 0
        largind = -1
        for i, frontier in enumerate(frontiers):
            if len(frontier) > larglen or largind == -1:
                larglen = len(frontier)
                largind = i
        return largind

    # returns next nav goal or None as a MoveBaseGoal
    def get_next_goal():
        if frontiers == None or centroids == None:
            return None

        centroid = centroids[get_next_frontier_index()]

        current_goal = MoveBaseGoal()
        
        current_goal.target_pose.header.frame_id = "map"
        current_goal.target_pose.header.stamp = rospy.Time.now()

        current_goal.target_pose.pose.position.x = centroid[0]
        current_goal.target_pose.pose.position.y = centroid[1]
        current_goal.target_pose.pose.position.z = 0

        current_goal.target_pose.pose.orientation.w = 1

        marker_array = MarkerArray()
        marker_array.markers = [make_marker(random.randint(0, 2147000000), centroid, (1,0,0), 0.25, 0)]
        marker_publisher.publish(marker_array)

        return current_goal

    # debug tranform from TFListener from T1
    def debug_transform(listener):
        # transform listener
        try:
            (pos, ori) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            rospy.loginfo('position {}'.format(pos))
            rospy.loginfo('orientation {}'.format(ori))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)

    while not rospy.is_shutdown():
        recalc_frontiers()

        # get next goal or None if None exist
        current_goal = get_next_goal()

        # debug transform
        debug_transform(listener)
        
        # update move goal if posssible
        if not current_goal == None:
            rospy.loginfo('starting goal {}'.format(current_goal))

            # send and wait for goal execution
            sac.send_goal(current_goal)
            wait_success = sac.wait_for_result()
            if not wait_success:
                rospy.logerr("error, no action server")
                break
            else:
                rospy.loginfo('navigation got result {}'.format(sac.get_result()))
        
        rospy.sleep(1)
