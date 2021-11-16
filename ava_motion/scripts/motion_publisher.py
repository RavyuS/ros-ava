#!/usr/bin/env python
import rospy
from ava_motion.msg import TrackedVehicle, Vehicle
import pandas as pd

t1 = pd.read_csv("/home/ravyu/catkin_ws/src/ava_motion/data/topic1.txt")
t2 = pd.read_csv("/home/ravyu/catkin_ws/src/ava_motion/data/topic2.txt")

t1_cnt = 0
t2_cnt = 0

self_pub = rospy.Publisher('topic_2', Vehicle)
other_pub = rospy.Publisher('topic_1', TrackedVehicle)


def publish_next_self():
    global t2_cnt
    row = t2.loc[t2_cnt]
    msg = Vehicle()
    msg.location.x = row["Lat"]
    msg.location.y = row[" Long"]
    msg.spd = row[" Speed(km/hr)"]
    msg.acc = row[" Acceleration(m/s)"]
    msg.angle = row[" Heading(radians)"]
    self_pub.publish(msg)
    t2_cnt+=1

def publish_next_other():
    global t1_cnt
    row = t1.loc[t1_cnt]
    msg = TrackedVehicle()
    msg.id = row["Object ID"]
    msg.type = row[" Object Type"]
    msg.vehicle.location.x = row[" dx(m)"]
    msg.vehicle.location.y = row[" dy(m)"]
    msg.vehicle.spd =row[" speed(m/s)"]
    msg.vehicle.acc = row[" acceleration(m/s2)"]
    other_pub.publish(msg)
    t1_cnt+=1







rospy.init_node('motion_publisher')

r = rospy.Rate(10) # 10hz
counter = 0
while not rospy.is_shutdown():
    if counter % 5 == 0:
        publish_next_self() # publish every 5 loops
    publish_next_other()
    counter +=1
    r.sleep()

