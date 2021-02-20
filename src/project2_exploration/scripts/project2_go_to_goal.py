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

if __name__ == '__main__':
    # targets to navigate to, in order
    #targets = [(-1,1,0),(-1,-1,0)]
    targets = []

    rospy.init_node('project2_go_to_goal', anonymous=True)

    listener = tf.TransformListener()

    marker_publisher = rospy.Publisher('frontier_markers', MarkerArray)

    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    sac.wait_for_server()

    occupancy_grid = None

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

    last_marker_ids = None

    # sends a request to remove all old markers
    def delete_old_markers():
        global last_marker_ids
        if last_marker_ids == None:
            return

        marker_array = MarkerArray()
        marker_array.markers = [make_marker(i, (0,0,0), (0,0,0), 1, 2) for i in last_marker_ids]
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

    # creates all markers for a given iteration
    def publish_markers(positions, labels, grid_info):
        global last_marker_ids
        delete_old_markers()

        unique_labels = list(np.unique(labels))

        # used for creating equidistant HSV colors
        deg_interval = 1.0 / len(unique_labels)

        #rospy.loginfo("deg_interval {}".format(deg_interval))

        marker_array = MarkerArray()

        hsv_colors = [(deg_interval * i, 1, 1) for i in range(len(unique_labels))]

        #rospy.loginfo("colors {}".format(hsv_colors))

        # build markers for frontier points
        marker_array.markers = [make_marker(i, grid_to_world(positions[i], grid_info), hsv_colors[unique_labels.index(labels[i])], 0.05, 0) for i in range(len(labels))]
        
        labels = np.array(labels)
        positions = np.array(positions)

        # calculate centroids
        # this line creates a 2d array where each element is all points of a specific frontier label, then maps those positions into world space, and then calculates each centroid
        centroids = [calculate_centroid([grid_to_world(position, grid_info) for position in frontier]) for frontier in [positions[labels == label] for label in unique_labels]]
        # append centroid markers to original markers
        marker_array.markers = marker_array.markers + [make_marker(len(marker_array.markers) + i, centroid, hsv_colors[i], 0.25, 0) for i, centroid in enumerate(centroids)]
        
        # extract marker ids for reference
        marker_ids = [marker.id for marker in marker_array.markers]
        last_marker_ids = marker_ids

        # publish markers
        marker_publisher.publish(marker_array)
        rospy.loginfo("published {} markers".format(len(marker_array.markers)))

    # callback for occupancy grid subscriber
    def on_occupancy_grid(data):
        global occupancy_grid
        # convert to frontier map
        # new_occ_grid = [100 if x == -1 else 0 for x in data.data]

        #new_occ_grid = [0 for i in range(0, len(data.data))]

        width = data.info.width
        height = data.info.height
        # functions to convert a given 1d grid position to a 2d coordinate pair part
        y = lambda i: i // data.info.width
        x = lambda i: i % data.info.width        

        # convert occupancy grid into frontier map occupancy grid
        new_occ_grid = [100 if is_empty_nearby(data.data, x(i), y(i), width, height) and data.data[i] == -1 else 0 for i in range(0, len(data.data))]

        occupancy_grid = new_occ_grid
        data2 = data
        data2.data = new_occ_grid
        frontiers_publisher.publish(data2)

        np_occ = np.array(occupancy_grid)

        # zip elements with their positions
        np_occ = np.array(zip(np_occ, [[x(i), y(i)] for i in range(0, len(new_occ_grid))]))
        # create a boolean mask for elements that are frontier elements
        np_occ_mapping = np.apply_along_axis(lambda x: x[0] == 100, 1, np_occ)
        # apply mask
        np_occ_filtered = np_occ[np_occ_mapping]
        del np_occ_mapping
        # map array to only the position
        np_occ = np.apply_along_axis(lambda x: x[1], 1, np_occ_filtered)
        del np_occ_filtered
        
        #rospy.loginfo(np_occ.shape)
        #rospy.loginfo("width: {}, height: {}".format(data.info.width, data.info.height))

        clustering = AgglomerativeClustering(linkage="ward", n_clusters=5)
        labels = clustering.fit_predict(np_occ)

        #rospy.loginfo(labels)
        
        publish_markers(np_occ, labels, data.info)

    rospy.Subscriber("/map", OccupancyGrid, on_occupancy_grid)
    frontiers_publisher = rospy.Publisher("/frontiers_map", OccupancyGrid)

    target_index = 0

    # returns next nav goal or None as a MoveBaseGoal
    def get_next_goal():
        global target_index

        if target_index > len(targets) - 1:
            return None

        current_target = targets[target_index]
        target_index += 1

        current_goal = MoveBaseGoal()
        
        current_goal.target_pose.header.frame_id = "map"
        current_goal.target_pose.header.stamp = rospy.Time.now()

        current_goal.target_pose.pose.position.x = current_target[0]
        current_goal.target_pose.pose.position.y = current_target[1]
        current_goal.target_pose.pose.position.z = current_target[2]

        current_goal.target_pose.pose.orientation.w = 1

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
