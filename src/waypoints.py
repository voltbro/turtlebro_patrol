import rospy
# XML parcer lib
import xml.etree.ElementTree as ET

#Msgs for points
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

#Transformations for points
import math
from tf.transformations import quaternion_from_euler

marker_id = 0

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

    global marker_id
    
    points = Marker()		
    points.header.frame_id = "/base_link"	# publish path in map frame		
    points.type = points.ARROW
    points.action = points.ADD
    points.lifetime = rospy.Duration(0)
    points.ns = "some_ns"
    points.id = marker_id
    marker_id += 1
    points.scale.x = 0.1
    points.scale.y = 0.01
    points.scale.z = 0.01	
    points.color.a = 1.0
    points.color.r = 0.0
    points.color.g = 1.0
    points.color.b = 0.0
    points.mesh_use_embedded_materials = True
    

    # using TF.transformation func to get quaternion from theta Euler angle
    q = quaternion_from_euler(0, 0, math.radians(float(goal[3])))

    
    points.pose.position.x = float(goal[1])
    points.pose.position.y = float(goal[2])
    points.pose.orientation.x = q[0]
    points.pose.orientation.y = q[1]
    points.pose.orientation.z = q[2]
    points.pose.orientation.w = q[3]


    #points.points.append(point)
    # Publish the MarkerArray
    return points


def main():
    rospy.init_node('echoer')
    publisher = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
    rospy.sleep(1)
    goals = goals_extractor_from_xml()
    for i in range(len(goals)):
        a = points_publisher(goals[i])
        print(a)
        publisher.publish(a)
    print("all done")

main()
rospy.spin()