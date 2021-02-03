#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


# utility class for simple vector math
class Vec3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "({},{},{})".format(self.x, self.y, self.z)

    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return self + other * -1

    def __mul__(self, scale):
        return Vec3(self.x * scale, self.y * scale, self.z * scale)
    
    def __div__(self, scale):
        if scale == 0:
            return Vec3(0,0,0)
        
        return self * (1/scale)
    
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
    
    def normalized(self):
        return self / self.magnitude()

    def bearing_to(self, other):
        return math.atan2(other.x - self.x, other.y - self.y)

# calculate total linear distance in vertices
def total_linear_dist(vertices):
    d = 0
    for i in range(0, len(vertices)):
        p1 = vertices[i]
        p2 = vertices[(i + 1)%len(vertices)]
        d += (p1 - p2).magnitude()
    
    return d

# class to abstract logic for moving a single segment
class SegmentInfo:
    def __init__(self, start_pos, end_pos, ticks_linear):
        rospy.loginfo("loading segment from {} to {} with {} linear ticks".format(start_pos, end_pos, ticks_linear))
        self.start_pos = start_pos
        self.end_pos = end_pos

        self.ticks_linear = ticks_linear

        self.elapsed_ticks = 0
    
    def tick(self):
        self.elapsed_ticks += 1

    def is_complete(self):
        return self.elapsed_ticks >= self.ticks_linear

    def vel(self, tick_time, linear_speed):
        #motion_scale = min(1, self.ticks_linear - self.elapsed_ticks)
        motion_scale = 1
        return (self.end_pos - self.start_pos).normalized() * linear_speed * tick_time * motion_scale

# estimated vertices for the D shape, will be used to compute velocities
vertices = [Vec3(0,0,0), Vec3(9,0,0), Vec3(10,-1,0), Vec3(10,-9,0), Vec3(9,-10,0), Vec3(0,-10,0), Vec3(0,-9,0), Vec3(.5,-9,0), Vec3(.5,-1,0), Vec3(0,-1,0), Vec3(0,0,0)]
# the scale of the shape being drawn
shape_scale = 20
# how many velocity updates occur each second
tick_rate = 50.0
# the length of each velocity udpate
tick_time = 1.0/tick_rate
# the desired runtime in seconds
runtime_seconds = 10


rospy.loginfo("ticktick is {}".format(tick_time))

if __name__ == '__main__':
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('du_garret_sugarbaker_d_velocity', anonymous=True)

    for i in range(0, len(vertices)):
        vertices[i] *= shape_scale

    rate = rospy.Rate(tick_rate)

    # adjust for runtime, runtime will be less than or equal to goal
    total_ticks_goal = runtime_seconds * tick_rate
    # calculate total travel distance to use in deriving segment tick counts
    _total_linear_dist = total_linear_dist(vertices)
    # calculate linear speed in order to make runtime less than or equal to goal length
    linear_speed = _total_linear_dist / total_ticks_goal * tick_rate

    rospy.loginfo("computed linear speed {}".format(linear_speed))

    segment_index = 0

    # returns the next SegmentInfo object to use
    def get_next_segment():
        global segment_index
        start_pos = vertices[segment_index]
        end_pos = vertices[(segment_index + 1)%len(vertices)]
        rospy.loginfo("next segment is from {} to {}".format(start_pos, end_pos))
        
        seg_ticks_linear = (end_pos - start_pos).magnitude() / linear_speed * tick_rate

        segment_index += 1
        return SegmentInfo(start_pos, end_pos, seg_ticks_linear)

    current_segment = get_next_segment()

    while not rospy.is_shutdown():
        # if the segment is complete get a new one
        if current_segment.is_complete():
            # if no new segments exist terminate program
            if segment_index == len(vertices):
                break
            current_segment = get_next_segment()
        
        # get current linear velocity to update
        vel = current_segment.vel(tick_time, linear_speed)

        msg = Twist()
        
        msg.linear.x = vel.x
        msg.linear.y = vel.y
        msg.linear.z = vel.z

        #rospy.loginfo("publishing twist " + str(msg))

        current_segment.tick()
        #publish twist
        pub.publish(msg)
        rate.sleep()


