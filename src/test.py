import rospy

from std_msgs.msg import String

rospy.init_node('echoer')
publisher = rospy.Publisher('/test1', String, queue_size = 10)
rospy.sleep(1)

a = String()
a.data = "123141234"
rospy.loginfo(a)
publisher.publish(a)
