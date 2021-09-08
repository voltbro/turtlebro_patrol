#!/usr/bin/env python3
import rospy
import math

#import Twist data type for direct control
from geometry_msgs.msg import Twist

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from std_msgs.msg import String

#Import standard Pose msg types and TF transformation to deal with quaternions
from tf.transformations import quaternion_from_euler

from pathlib import Path

# XML parser
import xml.etree.ElementTree as ET


class Patrol(object):

    def __init__(self):
        
        rospy.on_shutdown(self.on_shutdown)

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)        
        rospy.Subscriber('patrol_control', String, self.patrol_control_cb)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        self.patrol_mode = "home"

        self.current_point = 0
        self.home_point = [0, 0, 0, 'home']  # position x y theta of home 
        self.patrol_points = []
      
        self.waypoints_data_file = rospy.get_param('~waypoints_data_file', str(Path(__file__).parent.absolute()) + '/../data/goals.xml')

        self.fake_movement = True

        rospy.loginfo("Init done")


    def move(self):

        self.patrol_points = self.fetch_points(self.waypoints_data_file)

        # in that loop we will check if there is shutdown flag or rospy core have been crushed
        while not rospy.is_shutdown() :#and not self.shutdown:
            rospy.sleep(0.1)  # 10 Hz


    def on_shutdown(self):
        
        rospy.loginfo("Shutdown my patrol")
        self.cmd_pub.publish(Twist()) 
        self.client.action_client.stop()
        rospy.sleep(0.5)      
    
    def patrol_control_cb(self, message):

        if message.data in ("start", "pause", "resume", "home", "shutdown" ):  # to make sure command received is in list of commands

            rospy.loginfo("Patrol: {} sequence received".format(message.data))

            # stop robot at new message
            self.client.cancel_all_goals()
            self.cmd_pub.publish(Twist())

            if message.data == "start":
                self.patrol_mode = "patrol"
                self.patrol_command('start')

            if message.data == "pause":
                self.patrol_mode = "pause" 
             
            if message.data == "resume":
                self.patrol_mode = "patrol"
                self.patrol_command('current')

            if message.data == "home":
                self.patrol_mode = "home"
                self.patrol_command('home')

            if message.data == "shutdown":
                rospy.signal_shutdown("Have shutdown patrol_control command")

        else:
            rospy.loginfo("Patrol: Command unrecognized")

    def get_patrol_point(self, point_type):
        # point_type: [start current next home]

        if point_type == 'home':
            self.current_point = 0

        if point_type == 'start':
            self.current_point = 1  

        if point_type == 'next':
            self.current_point += 1
            #cycle patrol points
            if self.current_point >= len(self.patrol_points):
               self.current_point = 1 
            
        return self.patrol_points[self.current_point]

            
    def patrol_command(self, command):

        patrol_point = self.get_patrol_point(command)
        goal = self.goal_message_assemble(patrol_point)

        rospy.loginfo("Patrol: sending to move_base goal {} ".format(goal.target_pose.pose.position))
        
        # no real movement on debug mode 
        if self.fake_movement:
            rospy.sleep(5)
            self.move_base_cb(GoalStatus.SUCCEEDED, MoveBaseResult())
        else :
            # Waits until the action server has started up and started listening for goals.
            self.client.wait_for_server()
            self.client.send_goal(goal, done_cb=self.move_base_cb)


    def goal_message_assemble(self, point):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # Move to x, y meters of the "map" coordinate frame 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(point[0])
        goal.target_pose.pose.position.y = float(point[1])

        q = quaternion_from_euler(0, 0, math.radians(float(point[2]))) # using TF.transformation func to get quaternion from theta Euler angle
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Patrol: created goal from point {} ".format(point))
        return goal


    def move_base_cb(self, status, result):

        rospy.loginfo("Patrol: Goal reached {}".format(self.patrol_points[self.current_point][3]))
        
        # renew patrol point
        if(self.patrol_mode == "patrol"):
            # small pause in point
            rospy.sleep(1)
            self.patrol_command('next')

    def fetch_points(self, xml_file):

        rospy.loginfo("Patrol: XML Parsing started")

        try:
            # Define xml-goals file path
            tree = ET.parse(xml_file)  # get file from launch params
            root = tree.getroot()

            points = []
            points.append(self.home_point)

            # filling points
            for xml_element in root.findall('goal'):
                points.append([xml_element.get('x'), xml_element.get('y'), xml_element.get('theta'), xml_element.get('name')])

            rospy.loginfo("Patrol:  XML parcing done. goals detected:  {}".format(points))

            return points  

        except Exception as e:

            rospy.loginfo("XML parser failed")
            rospy.signal_shutdown("No valid XML file")


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('turtlebro_patroll')
        patrol = Patrol()
        patrol.move()


    except rospy.ROSInterruptException:
        patrol.set_shutdown()
        rospy.loginfo("Patrol stopped due to ROS interrupt")