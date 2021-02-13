#!/usr/bin/env python
# followed tutorial https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import rospy

if __name__ == '__main__':
    targets = [(-1,1,0),(-1,-1,0)]

    rospy.init_node('project2_go_to_goal', anonymous=True)

    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    sac.wait_for_server()

    target_index = 0

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

    while not rospy.is_shutdown():
        current_goal = get_next_goal()
        if current_goal == None:
            break
        
        rospy.loginfo('starting goal {}'.format(current_goal))

        sac.send_goal(current_goal)

        wait_success = sac.wait_for_result()
        if not wait_success:
            rospy.logerr("error, no action server")
            break
        else:
            rospy.loginfo('navigation got result {}'.format(sac.get_result()))
