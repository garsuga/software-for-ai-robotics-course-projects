#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import math
import os
import pickle
import random

if __name__ == '__main__':
    rospy.init_node('project3_wall_following', anonymous=True)
    
    vel_publisher = rospy.Publisher('/cmd_vel', Twist)

    to_rad = lambda x: (x % 360.0) / 360.0 * 2.0 * math.pi
    # | /
    # R -
    #
    # ^ rough sketch of angles
    # robot will follow walls to the right
    # negative angles will be wrapped
    # I might extend this later but 21 states is already a lot to define manually
    FRONT = to_rad(0)
    FRONT_RIGHT = to_rad(-45)
    RIGHT = to_rad(-90)

    probed_angles = [
        FRONT,
        FRONT_RIGHT,
        RIGHT
    ]

    debug_angle_names = [
        "FRONT",
        "FRONT_RIGHT",
        "RIGHT"
    ]

    # this value is for book-keeping and cannot be changed
    substates_per_angle = 3

    # distance to train robot to follow at
    ideal_wall_dist = .58
    # % of 1
    # tolerance for ideal distance
    wall_dist_tol = .2

    TOO_CLOSE = 0
    CORRECT = 1
    TOO_FAR = 2

    possible_substates = [
        TOO_CLOSE,
        CORRECT,
        TOO_FAR
    ]

    debug_substate_names = [
        "TOO_CLOSE",
        "CORRECT",
        "TOO_FAR"
    ]

    # generates a list of possible substates with the total length n and given possible values
    # differs from permutations because its length can exceed that of the possible values
    def substates_for(pref, n, possible):
        if n == 0:
            return [pref]
        sub = []
        for p in possible:
            sub += substates_for(pref + str(p), n - 1, possible)
        return sub

    # possible states are stored as strings
    # this order dictates the ordering of the q-table
    possible_states = substates_for("", len(probed_angles), possible_substates)

    # maps distance to the segment within the current state
    def dist_to_substate(dist):
        if dist < (1 - wall_dist_tol) * ideal_wall_dist:
            return TOO_CLOSE
        elif dist > (1 + wall_dist_tol) * ideal_wall_dist:
            return TOO_FAR
        return CORRECT

    # creates a state from a array of float distances
    def state_of(distances):
        substates = [str(dist_to_substate(dist)) for dist in distances]
        return "".join(substates)

    states = substates_per_angle ** len(probed_angles)

    # helper method to generate twist messages
    def twist_of(forward, angular):
        twist = Twist()
        twist.linear.x = forward
        twist.angular.z = angular
        return twist

    # speed values for scaling movement
    l_speed = .12
    a_speed = .15 * (2 * math.pi)
    mult_slow_rot = .3333
    
    # set of possible twist values
    # twist_of accepts signed forward, angular speeds
    # _I values are in-place rotation
    # _M values are rotation while moving forward
    FORWARD = twist_of(l_speed, 0)

    SLIGHT_RIGHT_M = twist_of(l_speed, -a_speed * mult_slow_rot)
    SLIGHT_RIGHT_I = twist_of(0, -a_speed * mult_slow_rot)
    RIGHT_M = twist_of(l_speed, -a_speed)
    RIGHT_I = twist_of(0, -a_speed)

    SLIGHT_LEFT_M = twist_of(l_speed, a_speed * mult_slow_rot)
    SLIGHT_LEFT_I = twist_of(0, a_speed * mult_slow_rot)
    LEFT_M = twist_of(l_speed, a_speed)
    LEFT_I = twist_of(0, a_speed)

    # array to hold possible action enums
    possible_actions = [
        FORWARD,
        SLIGHT_RIGHT_M,
        SLIGHT_RIGHT_I,
        RIGHT_M,
        RIGHT_I,
        SLIGHT_LEFT_M,
        SLIGHT_LEFT_I,
        LEFT_M,
        LEFT_I
    ]

    debug_action_names = [
        "FORWARD",
        "SLIGHT_RIGHT_INPLACE",
        "SLIGHT_RIGHT_MOVING",
        "RIGHT_MOVING",
        "RIGHT_INPLACE",
        "SLIGHT_LEFT_MOVING",
        "SLIGHT_LEFT_INPLACE",
        "LEFT_MOVING",
        "LEFT_INPLACE"
    ]

    # q_table is len(probed_angles) x len(actions)
    # this initializes the q-values to 0
    q_table = [([0] * len(possible_actions))[:] for i in range(states)]

    # substate is a dict of angle -> dist
    # global constants should be used
    # state is a string state
    def state_contains_substates(state, substates):
        for angle, dist in substates.items():
            angle_ordinal = probed_angles.index(angle)
            dist_ordinal = possible_substates.index(dist)
            if int(state[angle_ordinal]) != dist_ordinal:
                return False
        return True

    # creates q-values enforcing the given policy
    # substates is a dict of angle -> dist
    # ideal_action is a constant of the action intended to be used
    def q_policy(substates, ideal_action, value_if, value_else):
        ideal_action_ordinal = possible_actions.index(ideal_action)

        # iterate over action ordinals and their corresponding q_values
        for state_ordinal, state_q_values in enumerate(q_table):
            # iterate over state ordinals
            for action_ordinal in range(len(state_q_values)):
                # check if this state is within intended scope
                if state_contains_substates(possible_states[state_ordinal], substates):
                    if action_ordinal == ideal_action_ordinal:
                        state_q_values[action_ordinal] += value_if
                    else:
                        state_q_values[action_ordinal] += value_else

    # the following 'policies' describe predicates and ideal actions
    # these write to the q_table to give it initial values
    # default action is FORWARD

    def debug_q_table(q_table):
        message = "\n".join(
            [" ".join([str(e) for e in ele]) + " : " + possible_states[state_ordinal] 
            for state_ordinal, ele in enumerate(q_table)])
        return message

    def get_local_file_path(filename):
        return os.path.join(os.path.dirname(os.path.realpath(__file__)), filename)

    def write_table_to_file(filename):
        global q_table
        qtable_file = open(filename, "wb")
        pickle.dump(q_table, qtable_file)
        qtable_file.close()

    def read_table_file(filename):
        global q_table
        qtable_file = open(filename, "rb")
        q_table = pickle.load(qtable_file)
        qtable_file.close()

    #debug_q_table(q_table)
    #q_policy({
    #    RIGHT: TOO_FAR,
    #    FRONT: TOO_CLOSE
    #}, LEFT_I, 1, 0)
    #debug_q_table(q_table)

    # TODO: REMOVE TO END DEBUG
    #exit()

    qtable_path = get_local_file_path("q-table.dat")
    if not os.path.exists(qtable_path):
        # set default
        # matches all states and increments q-values for FORWARD actions by 1
        q_policy({}, FORWARD, .5, 0)

        # handle obstacle in-front, concave corner
        q_policy({
            FRONT: TOO_CLOSE,
            RIGHT: CORRECT
        }, LEFT_I, 1, 0)

        # handle obstacle in-front, concave corner, too close
        q_policy({
            FRONT: TOO_CLOSE,
            RIGHT: TOO_CLOSE
        }, LEFT_I, 1, 0)

        # in corner
        q_policy({
            FRONT: TOO_CLOSE,
            RIGHT: TOO_CLOSE,
            FRONT_RIGHT: TOO_CLOSE
        }, LEFT_I, 1, 0)

        # is facing slightly too right
        q_policy({
            RIGHT: TOO_CLOSE,
            FRONT_RIGHT: CORRECT
        }, SLIGHT_LEFT_M, 1, 0)

        # is facing much too right
        q_policy({
            RIGHT: TOO_CLOSE,
            FRONT_RIGHT: TOO_CLOSE
        }, LEFT_M, 1, 0)

        # is facing too left
        q_policy({
            RIGHT: TOO_FAR,
            FRONT_RIGHT: TOO_FAR
        }, RIGHT_M, 1, 0)

        # is facing too left
        q_policy({
            RIGHT: TOO_FAR,
            FRONT_RIGHT: CORRECT
        }, RIGHT_M, 1, 0)

        # is misplaced (cannot see wall)
        # spin and try to find wall. since this is probably due to it
        # turning too early we will look right
        # TODO: make move forward to keep from getting stuck??
        q_policy({
            RIGHT: TOO_FAR,
            FRONT_RIGHT: TOO_FAR,
            FRONT: TOO_FAR
        }, SLIGHT_RIGHT_M, 1, 0)

        # wall is ahead, not to side
        q_policy({
            RIGHT: TOO_FAR,
            FRONT: CORRECT
        }, SLIGHT_LEFT_M, 1, 0)

        # wall is ahead, too close, not to side
        q_policy({
            RIGHT: TOO_FAR,
            FRONT: TOO_CLOSE
        }, SLIGHT_LEFT_I, 1, 0)

        # approaching convex corner
        q_policy({
            RIGHT: CORRECT,
            FRONT: TOO_FAR,
            FRONT_RIGHT: TOO_FAR
        }, SLIGHT_RIGHT_M, 1, 0)

        # approaching convex corner too close
        q_policy({
            RIGHT: TOO_CLOSE,
            FRONT: TOO_FAR,
            FRONT_RIGHT: TOO_FAR
        }, FORWARD, 1, 0)

        # stuck on convex corner
        q_policy({
            RIGHT: TOO_FAR,
            FRONT: TOO_FAR,
            FRONT_RIGHT: TOO_CLOSE
        }, SLIGHT_LEFT_M, 2, 0)

        write_table_to_file(qtable_path)
    read_table_file(qtable_path)

    f = open(get_local_file_path("q-table.txt"), "w")
    f.write("q-values : state\n" + debug_q_table(q_table))
    f.write("\nindices correspond to {}".format(debug_action_names))
    f.close()

    last_state = None
    last_state_changed = False
    last_distances = None

    def on_laser_scan(data):
        #rospy.loginfo("Received laser data with {} points.".format(len(data.ranges)))

        angle_min = data.angle_min
        angle_increment = data.angle_increment
        angle_max = data.angle_max

        def get_at_angle(rad):
            for index in range(len(data.ranges)):
                # if the angle is the closest possible for the given interval
                if abs((angle_min + index * angle_increment) - rad) <= angle_increment * .5:
                    return data.ranges[index]
            raise Exception('Could not find angle {}'.format(rad))

        global last_state
        global last_distances
        last_distances = [get_at_angle(rad) for rad in probed_angles]
        last_state = state_of(last_distances)
        global last_state_changed
        last_state_changed = True

    rospy.Subscriber('/scan', LaserScan, on_laser_scan)

    def debug_state(state):
        message = "State: "
        for angle_ordinal, c in enumerate(state):
            distance_ordinal = int(c)
            message += "\n{}: {}".format(
                debug_angle_names[angle_ordinal], 
                debug_substate_names[distance_ordinal])
        rospy.loginfo(message)

    def get_random_action():
        return possible_actions[random.randrange(len(possible_actions))]

    # get Twist of best action based on state and q_table
    def get_best_action(state):
        actions_q_values = q_table[possible_states.index(state)]
        max_action = (0, FORWARD)

        for action_ordinal, q_value in enumerate(actions_q_values):
            if q_value > max_action[0]:
                max_action = (q_value, possible_actions[action_ordinal])

        return max_action[1]

    wait_time = .1

    elapsed_realtime = 0
    reset_simulation_callable = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    rate = rospy.Rate(5)

    wall_start_time = 0
    wall_travel_dist = 0
    away_start_time = 0
    last_save_time = 0
    prior_state = None

    wall_reward_per_time = .1

    max_bad_dist_time = 1
    min_el_bad_dist_time = 5

    is_by_wall = False

    epsilon = .1
    alpha = .3
    gamma = .5

    save_interval = 2

    def reward_action(last_state, next_state, action, reward):
        global q_table
        def get_qvalue(s, a):
            return q_table[possible_states.index(s)][possible_actions.index(a)]
        def set_qvalue(s, a, v):
            global q_table
            q_table[possible_states.index(s)][possible_actions.index(a)] = v
        def max_reward(state):
            m = 0
            for r in q_table[possible_states.index(state)]:
                m = max(r, m)
            return m
        set_qvalue(last_state, action, get_qvalue(last_state, action) + alpha * (reward + gamma * max_reward(next_state)) - get_qvalue(last_state, action))

    def correct_distance(distances):
        def close():
            # if the robot is too close on any dimension
            thresh = ideal_wall_dist * (1 - wall_dist_tol) * .75

            return distances[0] < thresh or distances[1] < thresh or distances[2] < thresh
        def far():
            # if the robot is not close enough on any side (lost wall)
            thresh = ideal_wall_dist * (1 + wall_dist_tol)
            return distances[2] > thresh
        return not (close() or far())

    def get_current_reward():
        if is_by_wall:
            return wall_reward_per_time * wall_travel_dist
        #else:
            #return -wall_reward_per_time * (elapsed_realtime - away_start_time)
        #return -wall_reward_per_time
        return 0

    def reset_simulation():
        global wall_start_time
        global away_start_time
        global elapsed_realtime
        global last_save_time
        global is_by_wall
        global prior_state
        global last_state
        global last_distances
        global last_state_changed
        global wall_travel_dist 

        wall_start_time = 0
        wall_travel_dist = 0
        away_start_time = 0
        elapsed_realtime = 0
        last_save_time = 0
        is_by_wall = False
        prior_state = None
        last_state = None
        last_distances = None
        last_state_changed = False
        reset_simulation_callable()

    action = None

    distance_travelling_actions = [FORWARD, RIGHT_M, SLIGHT_RIGHT_M, LEFT_M, SLIGHT_LEFT_M]

    while not rospy.is_shutdown():
        if elapsed_realtime - last_save_time >= save_interval:
            write_table_to_file(qtable_path)
            rospy.loginfo("Saving Q-Table")
            last_save_time = elapsed_realtime

        global last_state_changed
        global action
        global prior_state
        if last_state_changed:
            debug_state(last_state)
            last_state_changed = False

            if not prior_state == None:
                reward = get_current_reward()
                rospy.loginfo("\tReward is {}".format(reward))
                reward_action(prior_state, last_state, action, reward)

            if correct_distance(last_distances):
                if not is_by_wall:
                    is_by_wall = True
                    wall_start_time = elapsed_realtime
                    wall_travel_dist = 0
            else:
                if is_by_wall:
                    is_by_wall = False
                    away_start_time = elapsed_realtime
                    wall_travel_dist = 0
                elif elapsed_realtime > min_el_bad_dist_time and elapsed_realtime - away_start_time > max_bad_dist_time:
                    reset_simulation()
                    continue
            
            action = None

            if random.random() < epsilon:
                action = get_random_action()
            else:
                action = get_best_action(last_state)

            wall_travel_dist *= .8
            if is_by_wall and action in distance_travelling_actions:
                wall_travel_dist += 1
            

            rospy.loginfo("Chose state {}".format(debug_action_names[possible_actions.index(action)]))

            vel_publisher.publish(action)

            try:
                rate.sleep()
            except:
                rospy.loginfo("Simulation reset")
            elapsed_realtime += wait_time
            prior_state = last_state

        #else:
            #rospy.loginfo("Did not receive a laser scan since last action.")
