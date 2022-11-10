#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import json
import rospy
import threading
import signal
import subprocess
from subprocess import PIPE
import os
from utils.peer import *
from utils.service_client import *
from utils.media import *


def signal_handler(peer_id, token):
    def internal(signal, frame):
        # delete the peer object to exit
        request = delete_peer_request(peer_id, token)
        response = skyway_control(request)

    return internal


class EventListener(threading.Thread):
    _peer_id = ""
    _token = ""
    _is_running = True
    _pid_map = {}

    def __init__(self, peer_id, token):
        super().__init__()
        self._peer_id = peer_id
        self._token = token

    def run(self):
        has_data_config = rospy.has_param(rospy.get_name() + "/data")
        if not has_data_config:
            rospy.logdebug("no data config. ignore data events")

        has_media_config = rospy.has_param(rospy.get_name() + "/media")
        if not has_media_config:
            rospy.logdebug("no media config. ignore media events")

        while self._is_running:
            event = skyway_event(self._peer_id, self._token)
            if event is None:
                continue
            if "result" not in event:
                continue
            if "event" not in event["result"]:
                continue

            request_type = event["result"]["request_type"]
            if request_type == "PEER":
                self._peer_event_react(
                    event, has_data_config, has_media_config
                )
            elif request_type == "DATA":
                if has_data_config:
                    self._data_event_react(event)
            elif request_type == "MEDIA":
                if has_media_config:
                    self._media_event_react(event)

    def _peer_event_react(self, event, has_data_config, has_media_config):
        if event["result"]["event"] == "CLOSE":
            # Terminate this program when the release of the peer object is confirmed.
            self._is_running = False
            # SkyWay for ROSを落とす
            response = skyway_control(
                '{"request_type": "SYSTEM", "command": "SHUTDOWN"}'
            )
            # gStreamer Controllerを落とす
            (result, pid) = gst_launch("SYSTEM_EXIT", "", 0)
            rospy.signal_shutdown("finish")
        elif event["result"]["event"] == "CALL":
            if not has_media_config:
                return

            media_connection_id = event["result"]["call_params"][
                "media_connection_id"
            ]
            answer_query = create_answer_query(media_connection_id)
            response = skyway_control(answer_query)

    def _data_event_react(self, event):
        # media系のサンプルなのでdata系の処理は省略
        pass

    def _media_event_react(self, event):
        if not event["is_success"]:
            return

        if event["result"]["event"] == "STREAM":
            video_send_params = event["result"]["send_params"]["video"]
            video_send_ip = video_send_params["media"]["ip_v4"]
            video_send_port = video_send_params["media"]["port"]
            video_send_rtcp_ip = video_send_params["rtcp"]["ip_v4"]
            video_send_rtcp_port = video_send_params["rtcp"]["port"]

            audio_send_params = event["result"]["send_params"]["audio"]
            audio_send_ip = audio_send_params["media"]["ip_v4"]
            audio_send_port = audio_send_params["media"]["port"]
            audio_send_rtcp_ip = audio_send_params["rtcp"]["ip_v4"]
            audio_send_rtcp_port = audio_send_params["rtcp"]["port"]

            redirect_params = event["result"]["redirect_params"]
            video_redirect_ip = redirect_params["video"]["ip_v4"]
            video_redirect_port = redirect_params["video"]["port"]
            video_redirect_rtcp_ip = redirect_params["video_rtcp"]["ip_v4"]
            video_redirect_rtcp_port = redirect_params["video_rtcp"]["port"]

            audio_redirect_ip = redirect_params["audio"]["ip_v4"]
            audio_redirect_port = redirect_params["audio"]["port"]
            audio_redirect_rtcp_ip = redirect_params["audio_rtcp"]["ip_v4"]
            audio_redirect_rtcp_port = redirect_params["audio_rtcp"]["port"]

            script = rospy.get_param(rospy.get_name() + "/media/gst_script")
            script = script.replace(
                "SRC_VIDEO_RTP_PORT", str(video_redirect_port)
            )
            script = script.replace(
                "SRC_VIDEO_RTCP_PORT", str(video_redirect_rtcp_port)
            )
            script = script.replace(
                "SRC_AUDIO_RTP_PORT", str(audio_redirect_port)
            )
            script = script.replace(
                "SRC_AUDIO_RTCP_PORT", str(audio_redirect_rtcp_port)
            )
            script = script.replace(
                "DEST_VIDEO_RTP_PORT", str(video_send_port)
            )
            script = script.replace(
                "DEST_AUDIO_RTP_PORT", str(audio_send_port)
            )
            script = script.replace(
                "DEST_VIDEO_RTCP_PORT", str(video_send_rtcp_port)
            )
            script = script.replace(
                "DEST_AUDIO_RTCP_PORT", str(audio_send_rtcp_port)
            )
            script = script.replace("DEST", video_send_ip)

            (result, pid) = gst_launch("LAUNCH", script, 0)
            if result:
                self._pid_map[event["result"]["media_connection_id"]] = pid
        elif event["result"]["event"] == "CLOSE":
            media_connection_id = event["result"]["media_connection_id"]
            if media_connection_id in self._pid_map:
                pid = self._pid_map[media_connection_id]
                (result, pid) = gst_launch("EXIT", "", pid)
                self._pid_map.pop(media_connection_id)


def main():
    if not rospy.has_param(rospy.get_name() + "/peer_id"):
        rospy.logerr("no peer_id")
        exit(0)
    peer_id = rospy.get_param(rospy.get_name() + "/peer_id")
    target_id = rospy.get_param(rospy.get_name() + "/media/target_id")

    if "API_KEY" not in os.environ:
        rospy.logerr("no API_KEY")
        rospy.logerr("exiting")
        exit(0)

    key = os.environ["API_KEY"]
    request = create_peer_request(peer_id, key)
    peer_create_response = skyway_control(request)

    if not peer_create_response["is_success"]:
        return
    else:
        # succeed to create a peer object
        peer_id = peer_create_response["result"]["peer_id"]
        token = peer_create_response["result"]["token"]

        # hook ctrl-c to delete the peer object when exiting
        signal.signal(signal.SIGINT, signal_handler(peer_id, token))

        # 相手側にcallする
        create_call_message = create_call_query(peer_id, token, target_id)
        response = skyway_control(create_call_message)

        # this program only reacts to events
        event_thread = EventListener(peer_id, token)
        event_thread.start()

        rospy.spin()
        event_thread.join()


if __name__ == "__main__":
    rospy.init_node("my_node", log_level=rospy.INFO)
    main()
