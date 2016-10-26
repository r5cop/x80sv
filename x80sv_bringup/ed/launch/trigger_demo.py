#!/usr/bin/python

import rospy
from ed.srv import SimpleQuery
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped

def callback(msg):
    try:
        result = query(id=msg.data)
    except rospy.ServiceException as exc:
        rospy.logerr(exc)
        return

    if not result.entities:
        rospy.logwarn("I could not find entity '%s'"%msg.data)
        return

    e = result.entities[0]
    pub.publish(PoseStamped(pose=e.pose, header=Header(frame_id="/map", stamp=rospy.Time.now())))

    rospy.loginfo("Sending the following pose to move base: (x, y): (%f, %f)", e.pose.position.x, e.pose.position.y)

rospy.init_node('trigger_demo')
sub = rospy.Subscriber("/x80sv/trigger", String, callback, queue_size=10)
query = rospy.ServiceProxy('/x80sv/ed/simple_query', SimpleQuery)
pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

rospy.spin()
