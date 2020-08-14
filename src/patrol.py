#!/usr/bin/env python
import rospy
import math

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


class EmergencyReaction(object):

    def __init__(self):
        rospy.init_node('tb_ws_actions')
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Subscriber('patrol_control', String, self.patrol_control_command)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.log_pub = rospy.Publisher('/patrol_log', String, queue_size=10)
        self.goal_canceled = False  # flag for pause command
        self.go_home = False  # flag for home command
        self.pause = False
        self.next = False
        self.shutdown = False
        self.home_reached = False
        self.curent_goal = 0
        self.home_location = [0, 0, 0, 0]  # position where to go if "home" command recieved
        self.stop_cmd = Twist()
        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', '../data/goals.xml')
        self.data_loader()

        rospy.loginfo("Init done")
        self.log_pub.publish("Init done")

    def __del__(self):
        #deleting class
        self.shutdown()
    
    def patrol_control_command(self, alert):
        if alert.data in ("next", "shutdown", "pause", "home"): #to make sure command recieved is in list of commands
            print("%s sequence recieved" %alert.data)
            self.log_pub.publish("{} sequence recieved".format(alert.data))
            if alert.data == "next":
                self.next = True
                self.pause = False
                self.goal_canceled = False
                self.go_home = False
            elif alert.data == "pause":
                self.client.cancel_goal()
                self.next = False
                self.pause = True
                self.goal_canceled = True
                self.go_home = False
            elif alert.data == "home":
                self.client.cancel_goal()
                self.next = False
                self.pause = False
                self.goal_canceled = False
                self.go_home = True
            elif alert.data == "shutdown":
                self.client.cancel_goal()
                self.shutdown == True
                self.next = False
                self.pause = False
                self.goal_canceled = False
                self.go_home = False
            else:
                self.log_pub.publish("Command unrecognized")
                rospy.loginfo("Command unrecognized")

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
        self.log_pub.publish("patrol created goal {} from target {} ".format(goal, target))
        return goal

    def goal_reached(self):
            
        rospy.loginfo("Goal reached")
        self.log_pub.publish("Goal reached")

        if self.goal_canceled or self.pause:
            self.next = False
        elif self.go_home:
            self.go_home = False
        else:
            self.curent_goal += 1
            self.next = True

        rospy.sleep(0.5) # to make a little stop before next command
        

    def goal_send(self, goal_to_send):
        #self.log_pub.publish("patrol sending to move_base goal {} ".format(goal_to_send))
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

    def data_loader(self):
        rospy.loginfo("XML Parsing started")
        self.log_pub.publish("XML Parsing started")
        try:
            #Define xml-goals file path
            tree = ET.parse(self.waypoints_data_file) #get file from launch params
            root = tree.getroot()
            #constructing goals data variables
            self.goals = []
            #filing goals var with XML file data
            i = 0 #counter var
            for goal in root.findall('goal'):
                self.goals.append([]) #appending new list for new goal in goals's list
                self.goals[i].append(goal.get('id'))
                self.goals[i].append(goal.get('x'))
                self.goals[i].append(goal.get('y'))
                self.goals[i].append(goal.get('theta'))
                i += 1
            rospy.loginfo("XML parcing done. goals detected: {}".format(self.goals))
            self.log_pub.publish("XML parcing done. goals detected:  {}".format(self.goals))
        except:
            rospy.loginfo("XML parcer failed")
            self.log_pub.publish("XML parcer failed")

    def set_shutdown(self):
        self.log_pub.publish("Shutting down patrol")
        self.cmd_pub.publish(self.stop_cmd)  # it will be better if it can stop motors after shutdown
        rospy.signal_shutdown()

    def set_pause(self):
        self.cmd_pub.publish(self.stop_cmd)
    
    def going_home(self):
        self.go_home = True
        self.cmd_pub.publish(self.stop_cmd)
        self.goal_send(self.goal_message_assemble(self.home_location))

    def going_to_next_goal(self):
        self.next = False
        if self.curent_goal < len(self.goals):
            self.goal_send(self.goal_message_assemble(self.goals[self.curent_goal]))
        else:
            self.log_pub.publish("All goals achieved! Loop starts over")
            self.curent_goal = 0
            self.goal_send(self.goal_message_assemble(self.goals[self.curent_goal]))

    def controller(self):
        #main state machine controller
        if self.shutdown:
            self.set_shutdown()
        elif self.pause:
            self.set_pause()
        elif self.go_home:
            self.going_home()
        elif self.next:
            self.going_to_next_goal()
        rospy.sleep(0.2)
        self.controller()
            
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        er = EmergencyReaction()
        rospy.loginfo("Waiting for first command")
        er.controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        er.shutdown()
        rospy.loginfo("Patrol stoped due to ROS interrupt")