#!/usr/bin/env python3
import rospy
import math
import sys
import traceback
from pathlib import Path

#import Twist data type for direct control
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


class Patrol(object):

    def __init__(self):
        
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.on_shutdown(self.on_shutdown)

        rospy.Subscriber('patrol_control', String, self.patrol_control_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.goal_canceled = False  # flag for pause command
        self.go_home = False  # flag for home command
        self.pause = False
        self.next = False
        self.shutdown = False
        self.home_reached = False

        self.on_patrol = False

        self.current_goal = 0
        self.home_location = [0, 0, 0]  # position x y theta of home 
        self.stop_cmd = Twist()
        self.goals = {}
        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', str(Path(__file__).parent.absolute()) + '/../data/goals.xml')

        rospy.loginfo("Init done")


    def run(self):

        self.goals = self.fetch_goals(self.waypoints_data_file)

        rospy.loginfo("Waiting for first command")

        # in that loop we will check if there is shutdown flag or rospy core have been crushed
        while not rospy.is_shutdown() :#and not self.shutdown:
            if not self.shutdown:
                # sleep a little and call state machine handler
                rospy.sleep(0.1)  # 10 Hz
                self.controller()
            else:
                # we found shutdown flag, caused by user
                # time to stop robot and break that infinite loop
                self.set_shutdown()
                break

    def on_shutdown(self):
        rospy.loginfo("Shutdown my patrol")
        self.cmd_pub.publish(Twist())       
    
    def patrol_control_cb(self, message):

        if message.data in ("start", "next", "shutdown", "pause", "home"):  # to make sure command received is in list of commands

            rospy.loginfo("Patrol: {} sequence received".format(message.data))

            # stop if on duty
            if self.on_patrol == True:
                self.client.cancel_all_goals()
                self.cmd_pub.publish(self.stop_cmd)

            #start movement if not in patrol now
            if message.data == "start" and self.on_patrol == False :
                    self.on_patrol = True
                    self.current_goal = 1  

            if message.data == "next":
                self.current_goal = self.get_next_goal()

            if message.data == "pause":
                self.on_patrol = False

            if message.data == "home":
                self.current_goal = self.get_next_goal()

            if message.data == "shutdown":

        else:
            rospy.loginfo("Patrol: Command unrecognized")
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
        rospy.loginfo("Patrol: created goal {} from target {} ".format(goal, target))
        return goal

    def goal_reached(self):
            
        rospy.loginfo("Patrol: Goal reached {}".format(self.goals[self.current_goal]))

        if self.goal_canceled or self.pause:
            self.next = False
        elif self.go_home:
            self.go_home = False
        else:
            self.current_goal += 1
            self.next = True

        rospy.sleep(0.5) # to make a little stop before next command

    def goal_send(self, goal_to_send):
        rospy.loginfo("Patrol: sending to move_base goal {} ".format(goal_to_send))
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        # Sends the goal to the action server.
        self.client.send_goal(goal_to_send)
        self.client.wait_for_result()
        result = self.client.get_result()

        """
        while result == None or not self.goal_canceled: #"cb" fake cycle made
            result = self.client.get_result()
            rospy.sleep(0.2)
        """
        self.goal_reached()

    def fetch_goals(self, goals_file):

        rospy.loginfo("Patrol: XML Parsing started")

        try:
            # Define xml-goals file path
            tree = ET.parse(goals_file)  # get file from launch params
            root = tree.getroot()
            # constructing goals data variables

            goals = {}

            # filling goals var with XML file data
            for xml_element in root.findall('goal'):
                goals[int(xml_element.get('id'))] = [xml_element.get('x'), xml_element.get('y'), xml_element.get('theta')] 

            rospy.loginfo("Patrol:  XML parcing done. goals detected:  {}".format(goals))

            return goals  

        except Exception as e:
            rospy.logerr("XML parser failed {}".format(e))
            rospy.loginfo("XML parser failed")
            exc_info = sys.exc_info()
            traceback.print_exception(*exc_info)
            del exc_info

    def set_shutdown(self):
        rospy.loginfo("Patrol: shutting down patrol")
        rospy.signal_shutdown(reason="Patrol: shutdown caused by user")

    def set_pause(self):
        self.cmd_pub.publish(self.stop_cmd)
    
    def going_home(self):
        rospy.loginfo("Patrol: go home")
        self.cmd_pub.publish(self.stop_cmd)
        home_goal = self.goal_message_assemble(self.home_location)
        rospy.loginfo("Patrol: sending home_goal {}".format(home_goal))
        self.goal_send(home_goal)
        # self.go_home = False

    def going_to_next_goal(self):
        self.next = False
        if self.current_goal < len(self.goals):
            self.goal_send(self.goal_message_assemble(self.goals[self.current_goal]))
        else:
            rospy.loginfo("Patrol: All goals achieved! Starting patrolling again from first goal ")
            self.current_goal = 0
            self.goal_send(self.goal_message_assemble(self.goals[self.current_goal]))

    def controller(self):

        # shutdown will be parsed outside of that function, in main loop
        if self.pause:
            rospy.loginfo("Patrol: pause flag")
            self.set_pause()
        if self.go_home:
            rospy.loginfo("Patrol: go_home flag")
            self.going_home()
        if self.next:
            rospy.loginfo("Patrol: next flag")
            self.going_to_next_goal()
        rospy.sleep(0.2)


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('tb_patroll')
        patrol = Patrol()
        patrol.run()


    except rospy.ROSInterruptException:
        patrol.set_shutdown()
        rospy.loginfo("Patrol stopped due to ROS interrupt")