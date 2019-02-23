#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import random

def talker():
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('robot_coordinator', anonymous=True)
    rate = rospy.Rate(0.1) # every 10 sec
    while not rospy.is_shutdown():
        pose = get_random_pose_stamped()
        # rospy.loginfo(pose.pose.position.x,pose.pose.position.y)
        pub.publish(pose)
        rate.sleep()

def get_random_pose_stamped():
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = random.uniform(-1.8,1.8)
    pose.pose.position.y = random.uniform(-1.8,1.8)
    pose.pose.orientation.w = random.uniform(0.0,1.0)

    return pose

# def to_marker(self, robot):
#     marker = Marker()
#     marker.header.frame_id = "map"
#     marker.header.stamp = rospy.Time.now()
#     marker.pose.position.x = self.x
#     marker.pose.position.y = self.y
#     marker.pose.position.z = 0

#     quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
#     marker.pose.orientation.x = quaternion[0]
#     marker.pose.orientation.y = quaternion[1]
#     marker.pose.orientation.z = quaternion[2]
#     marker.pose.orientation.w = quaternion[3]

#     marker.scale.x = robot.width
#     marker.scale.y = robot.height
#     marker.scale.z = 0.2

#     marker.color.r = 1.0
#     marker.color.g = 0.0
#     marker.color.b = 0.0
#     marker.color.a = 1.0

#     marker.type = Marker.CUBE
#     marker.action = marker.ADD

#     return marker

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
