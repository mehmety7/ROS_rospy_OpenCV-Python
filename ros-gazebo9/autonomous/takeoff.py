#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import *
# ################################################################################################################################################
# -------------------------------------------------------------- GLOBAL DEFINITIONS ---------------------------------------------------
# ################################################################################################################################################




# ################################################################################################################################################
# -------------------------------------------------------------- STATES INSTRUCTIONS ---------------------------------------------------
# ################################################################################################################################################

def setArm():
    global waypoint_g

    set_destination(0, 0, 0, 0)
    for i in range(100):
        pose_pub.publish(waypoint_g)
        rospy.sleep(0.01)

    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" % e)


def setTakeoffMode(alt):
    global waypoint_g

    set_destination(0, 0, int(alt), 0)
    for i in range(100):
        pose_pub.publish(waypoint_g)
        rospy.sleep(0.01)

    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        takeoffService(altitude=int(alt), latitude=0,longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)


def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        takeoffService(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)


def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED')  # return true or false
    except rospy.ServiceException as e:
        print("service set_mode GUIDED call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)


# ################################################################################################################################################
# -------------------------------------------------------------- CALLBACK FUNCTIONS ----------------------------------------------------------
# ################################################################################################################################################

def globalAltitudeCallback(globalAltitudeCB):
    global alt
    alt = globalAltitudeCB.data


def globalStateCallback(globalStateCB):
    global okConnected
    global okArmed
    global okGuided
    okConnected = globalStateCB.connected
    okArmed = globalStateCB.armed
    okGuided = globalStateCB.guided

if __name__ == '__main__':

    rospy.init_node('takeoff_node', anonymous=True)

    rate = rospy.Rate(2.0)

    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, globalAltitudeCallback)
    rospy.Subscriber("/mavros/state", State, globalStateCallback)

    # Conecting check
    while not okConnected:
        rospy.sleep(0.01)

    # Guided check
    setGuidedMode()
    while not okGuided:
        rospy.sleep(0.01)
        setGuidedMode()

    print("Is Drone Guided:{}".format(okGuided))

    # Armed check
    setArm()
    while not okArmed:
        rospy.sleep(0.1)
        setArm()

    print("Drone Arm Status:{}".format(okArmed))


    # Altitude check
    h = 8.0
    setTakeoffMode(h)
    while alt < h - 0.2:
        rospy.sleep(0.01)
        
    setLandMode()
