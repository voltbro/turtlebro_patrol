#!/usr/bin/env python
import rospy

#import Twist data type for direct control
from geometry_msgs.msg import Twist

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

# XML parcer lib
import xml.etree.ElementTree as ET

class Emergency_reaction(object):

    def __init__(self):
        rospy.init_node('tb_ws_actions')
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Subscriber('patrol_control', String, self.patrol_control_alert)
        self.cmd_pub = rospy.Publisher('/cmd_ver', Twist, queue_size=1)
        self.paused = False #flag for pause command
        self.go_home = False #flag for home command
        self.current_goal = 0
        self.home = [0,0,1] #position where to go if "home" command recieved

        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', '../data/goals.xml')
        self.data_loader()

        rospy.loginfo("Init done")

    def __del__(self):
        #deleting class
        self.shutdown()

    def shutdown(self):
        rospy.loginfo('Shuting down patrol')
        rospy.signal_shutdown("Canceling goal")
        self.client.cancel_goal()
        rospy.signal_shutdown("Stoping robot")
        self.cmd_pub.publish(Twist)
        rospy.signal_shutdown("Shutting down rospy")
    
    def patrol_control_alert(self, alert):
        if alert.data in ("start", "stop", "pause", "resume", "home"): #to make sure command recieved is in list of commands
            print("%s sequence recieved" %alert.data)
            self.controller(alert.data)
        else:
            rospy.loginfo("Alert unrecognized")

    def goal_assemble(self, target):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move X meters forward along the x axis of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(target[1])
        goal.target_pose.pose.position.y = float(target[2])
        goal.target_pose.pose.orientation.w = float(target[3])
        return goal


    def goal_send(self, goal_to_send):
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        # Sends the goal to the action server.
        self.client.send_goal(goal_to_send)
        
        # Result of executing the action
        _result = self.client.get_result()
        while _result == None: #"cb" fake cycle made
            _result = self.client.get_result()
            rospy.sleep(0.2)

        rospy.sleep(0.5) #to make a little stop at goal point
        rospy.loginfo("Goal %s reached!" %self.current_goal)
        # Moving pointer to next goal
        self.current_goal += 1
        self.controller('start')  

    
    def data_loader(self):
        rospy.loginfo("XML Parsing started")
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
                self.goals[i].append(goal.get('w'))
                i += 1
            rospy.loginfo("XML parcing done. %s goals detected." %(len(self.goals)))
        except:
            rospy.loginfo("XML parcer failed")

    def controller(self, cmd):
        #main state machine controller
        if cmd == 'stop':
            self.client.cancel_goal()
            rospy.signal_shutdown("Shutting down")
        elif cmd == 'pause':
            self.paused = True
            self.current_goal -= 1
            self.client.cancel_goal()
        elif cmd == 'resume':
            self.paused = False
            self.client.cancel_goal()
            self.goal_send(self.goal_assemble(self.goals[self.current_goal]))
        elif cmd == 'home':
            self.go_home = True
            self.client.cancel_goal()
            self.goal_send(self.goal_assemble(self.home))
        else:
            while not self.paused and not self.go_home:
                if self.current_goal < len(self.goals):
                    self.goal_send(self.goal_assemble(self.goals[self.current_goal]))
                else:
                    self.current_goal = 0
                    self.goal_send(self.goal_assemble(self.goals[self.current_goal]))

    # If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        er = Emergency_reaction()
        rospy.loginfo("Waiting for Start command")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Patrol stoped due to ROS interrupt")