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


class EmergencyReaction(object):

    def __init__(self):
        rospy.init_node('tb_ws_actions')
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.Subscriber('patrol_control', String, self.patrol_control_alert)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.log_pub = rospy.Publisher('/patrol_log', String, queue_size=10)
        self.paused = False  # flag for pause command
        self.go_home = False  # flag for home command
        self.current_goal = 0
        self.home = [0, 0, 0, 1]  # position where to go if "home" command recieved
        self.stop_cmd = Twist()
        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', '../data/goals.xml')
        self.data_loader()

        rospy.loginfo("Init done")
        self.log_pub.publish("Init done")

    def __del__(self):
        #deleting class
        self.shutdown()

    def shutdown(self):
        print('Shutting down patrol')
        self.log_pub.publish("Shutting down patrol")
        print("Canceling goal")
        self.client.cancel_goal()
        # really navigation doesnt end here
        # its not good
        # TODO fix
        print("Stoping robot")
        self.cmd_pub.publish(self.stop_cmd)  # it will be better if it can stop motors after shutdown
    
    def patrol_control_alert(self, alert):
        if alert.data in ("start", "stop", "pause", "resume", "home"): #to make sure command recieved is in list of commands
            print("%s sequence recieved" %alert.data)
            self.log_pub.publish("{} sequence recieved".format(alert.data))
            self.controller(alert.data)
        else:
            self.log_pub.publish("Alert unrecognized")
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
        self.log_pub.publish("patrol created goal {} from target {} ".format(goal, target))
        return goal

    def goal_send(self, goal_to_send):
        self.log_pub.publish("patrol sending to move_base goal {} ".format(goal_to_send))
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        # Sends the goal to the action server.
        self.client.send_goal(goal_to_send)

        # Result of executing the action
        _result = self.client.get_result()

        while _result == None: #"cb" fake cycle made
            _result = self.client.get_result()
            rospy.sleep(0.2)
        self.log_pub.publish("result of sending goal is {} ".format(_result))

        rospy.sleep(0.5)  # to make a little stop at goal point
        self.log_pub.publish("Goal {} reached!".format(self.current_goal))
        rospy.loginfo("Goal %s reached!" %self.current_goal)
        # Moving pointer to next goal
        self.current_goal += 1
        self.controller('start')  

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
                self.goals[i].append(goal.get('w'))
                i += 1
            rospy.loginfo("XML parcing done. goals detected: {}".format(self.goals))
            self.log_pub.publish("XML parcing done. goals detected:  {}".format(self.goals))
        except:
            rospy.loginfo("XML parcer failed")
            self.log_pub.publish("XML parcer failed")

    def controller(self, cmd):
        self.log_pub.publish("controller of patrol")
        #main state machine controller
        if cmd == 'stop':
            self.shutdown()
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
                    self.log_pub.publish("All goals achieved! Loop starts again")
                    self.current_goal = 0
                    self.goal_send(self.goal_assemble(self.goals[self.current_goal]))


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        er = EmergencyReaction()
        rospy.loginfo("Waiting for Start command")
        rospy.spin()
    except rospy.ROSInterruptException:
        er.shutdown()
        rospy.loginfo("Patrol stoped due to ROS interrupt")
