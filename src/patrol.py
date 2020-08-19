#!/usr/bin/env python
import rospy
import math
import sys
import traceback
import pdb

# import Twist data type for direct control
from geometry_msgs.msg import Twist

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

#Import standard Pose msg types and TF transformation to deal with quaternions
from tf.transformations import quaternion_from_euler

# XML parser lib
import xml.etree.ElementTree as ET


class PatrolHandler(object):
    def __init__(self):
        rospy.init_node('patrol_handler', log_level=rospy.DEBUG)
        # TODO create only one state
        # TODO remove prints, remove pdb
        # TODO add target points to rviz
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # subs and pubs
        rospy.Subscriber('patrol_control', String, self.patrol_control_command)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # robot must wait by default
        self.state = "wait"
        # state can be "wait" or "move"
        self.home_target = [0, 0, 0, 0]
        # position where to go if "home" command received
        self.stop_cmd = Twist()
        # command to stop motors
        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', '../data/goals.xml')
        # call load data from xml-file
        self.targets = list()
        self.data_loader()
        # lets set current goal as 0`s element from goals array, loaded by self.data_loader()
        self.current_target_num = 0
        self.current_target = self.targets[self.current_target_num]
        # full list of commands
        self.allowed_user_commands = ["next", "shutdown", "pause", "home"]
        # default user cmd is undefined and marked as done,
        self.current_user_cmd = "done"

        rospy.loginfo("Init done, go to loop")
        # go to infinite loop
        self.loop()

    def loop(self):
        try:
            # in that loop we will infinitely check user cmd, if it was updated, we handle it
            while not rospy.core.is_shutdown():
                if self.current_user_cmd == "shutdown":
                    rospy.logdebug("Patrol: shutdown flag")
                    # we found shutdown flag, caused by user
                    # dont care what state it is, just do shutdown
                    # time to stop robot
                    self.cmd_pub.publish(self.stop_cmd)
                    # and break that infinite loop
                    rospy.signal_shutdown(reason="Patrol: shutdown caused by user")
                    break

                if self.current_user_cmd == "pause":
                    rospy.logdebug("Patrol: pause flag")
                    # cancel current goal
                    self.client.cancel_goal()
                    # change the state no matter what it was before
                    self.state = "wait"
                    # mark user command as done
                    self.current_user_cmd = "done"
                    # stop motors
                    self.cmd_pub.publish(self.stop_cmd)

                if self.current_user_cmd == "home":
                    rospy.logdebug("Patrol: go_home flag")
                    # cancel current goal
                    self.client.cancel_goal()
                    # stop motors
                    self.cmd_pub.publish(self.stop_cmd)
                    # add new goal - home
                    self.current_target = self.home_target
                    home_goal = self.goal_message_assemble(self.home_target)
                    rospy.logdebug("Patrol: sending home_goal {}".format(home_goal))
                    self.goal_send(home_goal)

                if self.current_user_cmd == "next":
                    rospy.logdebug("Patrol: next flag")
                    self.state = "move"

                    if self.current_target_num < len(self.targets):
                        rospy.logdebug("Patrol: self.current_target_num < len(self.targets)")
                        self.current_target_num += 1
                        self.current_target = self.targets[self.current_target_num]
                        rospy.loginfo("Patrol: next target is {}".format(self.current_target))
                        self.goal_send(self.goal_message_assemble(self.current_target))
                    else:
                        rospy.logdebug("Patrol: All goals achieved! Start patrolling again from first goal ")
                        self.current_target_num = 0
                        self.goal_send(self.goal_message_assemble(self.targets[self.current_target_num]))

                rospy.sleep(0.1)  # 10 Hz

        except KeyboardInterrupt:
            rospy.logdebug("Patrol: keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('Patrol: keyboard interrupt')
    
    def patrol_control_command(self, alert_msg):
        # pdb.set_trace()
        if alert_msg.data in self.allowed_user_commands:
            # to make sure received command is in list of commands
            rospy.loginfo("Patrol: {} sequence received".format(alert_msg.data))
            self.current_user_cmd = alert_msg.data
            # if alert_msg.data == "next":
            #     self.next_ = True
            #     self.pause = False
            #     self.goal_canceled = False
            #     self.go_home = False
            # elif alert_msg.data == "pause":
            #     self.client.cancel_goal()
            #     self.next_ = False
            #     self.pause = True
            #     self.goal_canceled = True
            #     self.go_home = False
            # elif alert_msg.data == "home":
            #     # pdb.set_trace()
            #     self.client.cancel_goal()
            #     self.next_ = False
            #     self.pause = False
            #     self.goal_canceled = True
            #     self.go_home = True
            # elif alert_msg.data == "shutdown":
            #     self.client.cancel_goal()
            #     self.shutdown = True
            #     self.next_ = False
            #     self.pause = False
            #     self.goal_canceled = False
            #     self.go_home = False
        else:
            rospy.loginfo("Patrol: Command unrecognized '{}'".format(alert_msg.data))

    def goal_message_assemble(self, target):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move to x, y meters of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(target[1])
        goal.target_pose.pose.position.y = float(target[2])
        q = quaternion_from_euler(0, 0, math.radians(float(target[3]))) # using TF.transformation func to get quaternion from theta Euler angle
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        rospy.logdebug("Patrol: created goal {} from target {} ".format(goal, target))
        return goal

    # def goal_reached(self):
    #     # pdb.set_trace()
    #     rospy.loginfo("Patrol: Goal reached {}".format(self.targets[self.current_target_num]))
    #
    #     if self.goal_canceled or self.pause:
    #         self.next_ = False
    #         return
    #     if self.go_home:
    #         self.go_home = False
    #         return
    #
    #     self.current_target_num += 1
    #     self.next_ = True
    #
    #     rospy.sleep(0.5)  # to make a little stop before next command

    def goal_send(self, goal_to_send):

        rospy.logdebug("Patrol: sending to move_base goal {} ".format(goal_to_send))
        # Waits until the action server has started up and started listening for goals.
        rospy.logdebug("Patrol: self.client.wait_for_server()")
        self.client.wait_for_server()

        # Sends the goal to the action server.
        rospy.logdebug("Patrol: self.client.send_goal(goal_to_send)")
        self.client.send_goal(goal_to_send)
        rospy.logdebug("Patrol: self.client.wait_for_result()")
        self.client.wait_for_result()

        result = self.client.get_result()
        rospy.logdebug("Patrol: self.client.get_result() : {}".format(result))

        # while result is None or not self.goal_canceled: #"cb" fake cycle made
        #     result = self.client.get_result()
        #     rospy.sleep(0.2)


        # now goal reached, time to handle it
        rospy.logdebug("Patrol: self.goal_reached()")
        rospy.loginfo("Patrol: Goal reached {}".format(self.targets[self.current_target_num]))

        # lets check if we were on a way to home
        if self.current_target == self.home_target:
            # it means that we dont need to set new goal, just wait
            self.state = "wait"
            return
        else:
            # if we were not on a way to home,
            # check if the user has already changed the state from 'move' to 'wait'
            # if state is "wait", it means that we do not need to send new goal
            if self.state == "wait" :
                return

            if self.state == "move":
                # we have to move

                # send next command to itself
                self.current_user_cmd = "next"
                return



        rospy.sleep(0.5)
        # to make a little stop before next command

    def data_loader(self):
        rospy.loginfo("Patrol: XML Parsing started")
        rospy.logdebug("Patrol: XML Parsing started")
        try:
            # Define xml-goals file path
            tree = ET.parse(self.waypoints_data_file)  # get file from launch params
            root = tree.getroot()
            # constructing goals data variables

            # filling goals var with XML file data
            i = 0  # counter var
            for goal in root.findall('goal'):
                self.targets.append(list())  # appending new list for new goal in goals's list
                self.targets[i].append(goal.get('id'))
                self.targets[i].append(goal.get('x'))
                self.targets[i].append(goal.get('y'))
                self.targets[i].append(goal.get('theta'))
                i += 1
            rospy.loginfo("Patrol: XML parcing done. goals detected: {}".format(self.targets))
            rospy.logdebug("Patrol:  XML parcing done. goals detected:  {}".format(self.targets))
        except Exception as e:
            rospy.logerr("XML parser failed {}".format(e))
            rospy.logdebug("XML parser failed")
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)
            del exc_info

    # def set_shutdown(self):
    #     rospy.logdebug("Patrol: shutting down patrol")
    #     self.cmd_pub.publish(self.stop_cmd)  # it will be better if it can stop motors after shutdown
    #     rospy.signal_shutdown(reason="Patrol: shutdown caused by user")
    
    # def going_home(self):
    #     rospy.logdebug("Patrol: go home")
    #     rospy.logdebug("Patrol: stopping robot")
    #     self.cmd_pub.publish(self.stop_cmd)
    #     home_goal = self.goal_message_assemble(self.home_location)
    #     rospy.logdebug("Patrol: sending home_goal {}".format(home_goal))
    #     self.goal_send(home_goal)
    #     # self.go_home = False

    # def going_to_next_goal(self):
    #     self.next_ = False
    #     if self.current_target_num < len(self.targets):
    #         self.goal_send(self.goal_message_assemble(self.targets[self.current_target_num]))
    #     else:
    #         rospy.logdebug("Patrol: All goals achieved! Starting patrolling again from first goal ")
    #         self.current_target_num = 0
    #         self.goal_send(self.goal_message_assemble(self.targets[self.current_target_num]))


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    ph = PatrolHandler()
    rospy.loginfo("Waiting for first command")

