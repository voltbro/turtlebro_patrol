#!/usr/bin/env python
import rospy
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
        self.current_goal = 0
        self.home = [0,0,0]

        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', '../data/goals.xml')
        self.data_loader()

        rospy.loginfo("Init done")
    
    def patrol_control_alert(self, alert):
        if alert.data == "start":
            self.started = True
            rospy.loginfo("Start sequence recieved")
            self.controller(alert.data)
        elif alert.data == "stop":
            self.started = False
            rospy.loginfo("STOP sequence recieved")
            self.controller(alert.data)
        elif self.started:
            self.controller(alert.data)
        else:
            rospy.loginfo("TB stopped")

    def goal_assemble(self, target):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move X meters forward along the x axis of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(target[1])
        goal.target_pose.pose.position.y = float(target[2])
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1.0
        return goal


    def goal_send(self, goal_to_send):
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        # Sends the goal to the action server.
        self.client.send_goal(goal_to_send)
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result() #TODO rework for CB driven architecture
        # Result of executing the action
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.loginfo("Goal NOT reached!")
        else:
            rospy.loginfo("Goal reached!")
            self.current_goal += 1
            self.controller('start')
    
    def data_loader(self):
        try:
            #Define xml-goals file path
            tree = ET.parse(self.waypoints_data_file)
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
                i += 1
            rospy.loginfo("XML parcing done. %s goals detected." %(len(self.goals)))
        except:
            rospy.loginfo("XML parcer failed")

    def controller(self, cmd):
        if cmd == 'stop':
            rospy.loginfo("STOP command recieved")
            self.client.cancel_goal()
            rospy.signal_shutdown("Shutting down")
        elif cmd == 'pause':
            rospy.loginfo("Pause command recieved")
            self.client.cancel_goal()
        elif cmd == 'resume':
            rospy.loginfo("Resume command recieved")
            self.goal_send(self.goal_assemble(self.goals[self.current_goal]))
        elif cmd == 'home':
            rospy.loginfo("GO HOME command recieved")
            self.client.cancel_goal()
            self.started = False
            self.goal_send(self.goal_assemble(self.home))
        elif cmd == 'start':
            while self.started:
                if self.current_goal < len(self.goals):
                    self.goal_send(self.goal_assemble(self.goals[self.current_goal]))
                else:
                    self.current_goal = 0
                    self.goal_send(self.goal_assemble(self.goals[self.current_goal]))
        else:
            self.client.cancel_goal()
            rospy.loginfo("Service STOPPED")

    # If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        er = Emergency_reaction()
        rospy.loginfo("Waiting for Start command")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Thermovisor Alert stopped")