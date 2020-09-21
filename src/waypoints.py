import rospy
# XML parcer lib
import xml.etree.ElementTree as ET

#Msgs for points
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

#Transformations for points
from tf.transformations import quaternion_from_euler

rospy.init_node('echoer')

def goals_extractor_from_xml():
    rospy.loginfo("XML Parsing started")
    try:
        waypoints_data_file = rospy.get_param('~waypoints_data_file', '../data/goals.xml')
        #Define xml-goals file path
        tree = ET.parse(waypoints_data_file) #get file from launch params
        root = tree.getroot()
        #constructing goals data variables
        goals = []
        #filing goals var with XML file data
        i = 0 #counter var
        for goal in root.findall('goal'):
            goals.append([]) #appending new list for new goal in goals's list
            goals[i].append(goal.get('id'))
            goals[i].append(goal.get('x'))
            goals[i].append(goal.get('y'))
            goals[i].append(goal.get('theta'))
            i += 1
        rospy.loginfo("XML parcing done. goals detected: {}".format(goals))

    except:
        rospy.loginfo("XML parcer failed")
    
    return goals


def points_publisher(goal):
    publisher = rospy.Publisher('visualization_marker', Marker, queue_size = 10)
    points = Marker()		
    points.header.frame_id = "/map"	# publish path in map frame		
    points.type = points.POINTS
    points.action = points.ADD
    points.lifetime = rospy.Duration(0)
    points.id = marker_id
    marker_id += 1
    points.scale.x = 0.1
    points.scale.y = 0.1	
    points.color.a = 1.0
    points.color.r = 0.0
    points.color.g = 0.0
    points.color.b = 1.0

    # using TF.transformation func to get quaternion from theta Euler angle
    q = quaternion_from_euler(0, 0, math.radians(float(goal[3])))

    point = Point()
    point.pose.position.x = float(goal[1])
    point.pose.position.y = float(goal[2])
    point.pose.orientation.x = q[0]
    point.pose.orientation.y = q[1]
    point.pose.orientation.z = q[2]
    point.pose.orientation.w = q[3]


    points.points.append(point)
    # Publish the MarkerArray
    publisher.publish(points)


def main()
   goals = goals_extractor_from_xml()
   for i in len(goals):
       points_publisher(goal[i])
    print("all done")