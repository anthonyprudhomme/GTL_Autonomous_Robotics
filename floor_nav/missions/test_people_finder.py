#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

scale=2.0
vel=0.5

tc.WaitForAuto()
try:
    tc.Wander()
    # Start the wait for roi task in the background
    w4face = tc.WaitForFace(foreground=False,roi_x=1.,roi_y=6.,roi_radius=1.0)
    # Prepare a condition so that the following gets executed only until the 
    # Region of Interest is found
    tc.addCondition(ConditionIsCompleted("Face detector",tc,w4face))

    try:
        tc.Wander()
        
    except TaskConditionException, e:
        rospy.loginfo("Wandering interrupted on condition: %s")
        # This means the conditions were triggered. We need to react to it
        # Conditions are cleared on trigger
        tc.StareAtFace()
		tc.SetHeading(target = 180, relative = True)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
