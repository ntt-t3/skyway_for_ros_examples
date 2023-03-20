# -*- coding: utf-8 -*-

import json
import rospy
from skyway.srv import *
from gstreamer_launcher.srv import *


def skyway_control(json_str):
    rospy.wait_for_service("skyway_control")
    try:
        controller = rospy.ServiceProxy("skyway_control", SkyWayControl)
        response = controller(json_str)
        return json.loads(response.response)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


def skyway_event(peer_id, token):
    rospy.wait_for_service("skyway_events")
    try:
        recv_event = rospy.ServiceProxy("skyway_events", SkyWayEvents)
        event = recv_event()
        return json.loads(event.response)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)


def gst_launch(message_type, command, pid):
    rospy.wait_for_service("gst_launch")
    try:
        controller = rospy.ServiceProxy("gst_launch", GStreamerLauncher)
        result = controller(message_type, command, pid)
        return (result.result, result.pid)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
