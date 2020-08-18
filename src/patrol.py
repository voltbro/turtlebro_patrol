#!/usr/bin/env python
import rospy
import math
import sys
import traceback
import pdb

#import Twist data type for direct control
from geometry_msgs.msg import Twist

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

#Import standard Pose msg types and TF transformation to deal with quaternions
from tf.transformations import quaternion_from_euler

# XML parcer lib
import xml.etree.ElementTree as ET


class PatrolHandler(object):

    def __init__(self):
        rospy.init_node('patrol_handler')
        # TODO remove log pub
        # TODO create only one state 
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # subs and pubs
        rospy.Subscriber('patrol_control', String, self.patrol_control_command)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.log_pub = rospy.Publisher('/patrol_log', String, queue_size=10)

        self.state = "wait"
        # state can be "wait" or "move" or "need_shutdown"
        self.home_location = [0, 0, 0, 0]
        # position where to go if "home" command recieved
        self.stop_cmd = Twist()
        # command to stop motors
        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', '../data/goals.xml')
        # call load data from xml-file
        self.goals = list()
        self.data_loader()
        # lets set current goal as 0`s element from goals array, loaded by self.data_loader()
        self.current_goal_num = 0
        self.current_goal = self.goals[self.current_goal_num]

        rospy.loginfo("Init done, go to loop")
        self.log_pub.publish("Init done, go to loop")
        # go to infinite loop
        self.loop()

    def loop(self):
        try:
            # in that loop we will check if there is shutdown flag or rospy core have been crushed
            while not rospy.core.is_shutdown():
                if self.state == "need_shutdown":
                    self.log_pub.publish("Patrol: shutdown flag")
                    # we found shutdown flag, caused by user
                    # time to stop robot and break that infinite loop
                    self.set_shutdown()
                    break
                if self.state == "need_shutdown":
                    self.log_pub.publish("Patrol: pause flag")
                    self.set_pause()
                if self.go_home:
                    self.log_pub.publish("Patrol: go_home flag")
                    self.going_home()
                if self.next_:
                    self.log_pub.publish("Patrol: next flag")
                    self.going_to_next_goal()
                rospy.sleep(0.2)  # 5 Hz
        except KeyboardInterrupt:
            self.log_pub.publish("Patrol: keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('Patrol: keyboard interrupt')
    
    def patrol_control_command(self, alert):
        # pdb.set_trace()
        if alert.data in ("next", "shutdown", "pause", "home"):  # to make sure command recieved is in list of commands
            print("Patrol: {} sequence received".format(alert.data))
            self.log_pub.publish("Patrol: {} sequence received".format(alert.data))
            if alert.data == "next":
                self.next_ = True
                self.pause = False
                self.goal_canceled = False
                self.go_home = False
            elif alert.data == "pause":
                self.client.cancel_goal()
                self.next_ = False
                self.pause = True
                self.goal_canceled = True
                self.go_home = False
            elif alert.data == "home":
                # pdb.set_trace()
                self.client.cancel_goal()
                self.next_ = False
                self.pause = False
                self.goal_canceled = True
                self.go_home = True
            elif alert.data == "shutdown":
                self.client.cancel_goal()
                self.shutdown = True
                self.next_ = False
                self.pause = False
                self.goal_canceled = False
                self.go_home = False
        else:
            self.log_pub.publish("Patrol: Command unrecognized")
            rospy.loginfo("Patrol: Command unrecognized")

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
        self.log_pub.publish("Patrol: created goal {} from target {} ".format(goal, target))
        return goal

    def goal_reached(self):
        # pdb.set_trace()
        rospy.loginfo("Patrol: Goal reached {}".format(self.goals[self.current_goal_num]))
        self.log_pub.publish("Patrol: Goal reached {}".format(self.goals[self.current_goal_num]))

        if self.goal_canceled or self.pause:
            self.next_ = False
            return
        if self.go_home:
            self.go_home = False
            return

        self.current_goal_num += 1
        self.next_ = True

        rospy.sleep(0.5) # to make a little stop before next command

    def goal_send(self, goal_to_send):
        # pdb.set_trace()
        self.log_pub.publish("Patrol: sending to move_base goal {} ".format(goal_to_send))
        # Waits until the action server has started up and started listening for goals.
        self.log_pub.publish("Patrol: self.client.wait_for_server()")
        self.client.wait_for_server()
        # Sends the goal to the action server.
        self.log_pub.publish("Patrol: self.client.send_goal(goal_to_send)")
        self.client.send_goal(goal_to_send)
        self.log_pub.publish("Patrol: self.client.wait_for_result()")
        self.client.wait_for_result()
        # self.log_pub.publish("Patrol: XML Parsing started")
        result = self.client.get_result()
        self.log_pub.publish("Patrol: self.client.get_result() : {}".format(result))

        # while result is None or not self.goal_canceled: #"cb" fake cycle made
        #     result = self.client.get_result()
        #     rospy.sleep(0.2)

        self.log_pub.publish("Patrol: self.goal_reached()")
        self.goal_reached()

    def data_loader(self):
        rospy.loginfo("Patrol: XML Parsing started")
        self.log_pub.publish("Patrol: XML Parsing started")
        try:
            # Define xml-goals file path
            tree = ET.parse(self.waypoints_data_file)  # get file from launch params
            root = tree.getroot()
            # constructing goals data variables

            # filling goals var with XML file data
            i = 0  # counter var
            for goal in root.findall('goal'):
                self.goals.append(list())  # appending new list for new goal in goals's list
                self.goals[i].append(goal.get('id'))
                self.goals[i].append(goal.get('x'))
                self.goals[i].append(goal.get('y'))
                self.goals[i].append(goal.get('theta'))
                i += 1
            rospy.loginfo("Patrol: XML parcing done. goals detected: {}".format(self.goals))
            self.log_pub.publish("Patrol:  XML parcing done. goals detected:  {}".format(self.goals))
        except Exception as e:
            rospy.logerr("XML parser failed {}".format(e))
            self.log_pub.publish("XML parser failed")
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)
            del exc_info

    def set_shutdown(self):
        self.log_pub.publish("Patrol: shutting down patrol")
        self.cmd_pub.publish(self.stop_cmd)  # it will be better if it can stop motors after shutdown
        rospy.signal_shutdown(reason="Patrol: shutdown caused by user")

    def set_pause(self):
        self.cmd_pub.publish(self.stop_cmd)
    
    def going_home(self):
        self.log_pub.publish("Patrol: go home")
        self.log_pub.publish("Patrol: stopping robot")
        self.cmd_pub.publish(self.stop_cmd)
        home_goal = self.goal_message_assemble(self.home_location)
        self.log_pub.publish("Patrol: sending home_goal {}".format(home_goal))
        self.goal_send(home_goal)
        # self.go_home = False

    def going_to_next_goal(self):
        self.next_ = False
        if self.current_goal_num < len(self.goals):
            self.goal_send(self.goal_message_assemble(self.goals[self.current_goal_num]))
        else:
            self.log_pub.publish("Patrol: All goals achieved! Starting patrolling again from first goal ")
            self.current_goal_num = 0
            self.goal_send(self.goal_message_assemble(self.goals[self.current_goal_num]))


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    ph = PatrolHandler()
    rospy.loginfo("Waiting for first command")

