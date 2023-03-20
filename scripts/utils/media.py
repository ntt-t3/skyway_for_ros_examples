# -*- coding: utf-8 -*-

import json
import socket
from contextlib import closing


def find_free_port():
    with closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.bind(("", 0))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return s.getsockname()[1]


def create_call_query(peer_id, token, target_id):
    video_port = find_free_port()
    video_rtcp_port = find_free_port()
    audio_port = find_free_port()
    audio_rtcp_port = find_free_port()

    answer_query = (
        "{"
        '    "request_type":"MEDIA",'
        '    "command":"CALL",'
        '    "params":{'
        '       "peer_id":"%s",'
        '       "token":"%s",'
        '       "target_id":"%s",'
        '        "constraints":{'
        '            "video_params":{'
        '                "band_width":1500,'
        '                "codec":"H264",'
        '                "payload_type":96,'  # ここで指定するpayload_typeはgStreamerで指定するものと一致させること。番号はRFCに沿っていれば何番でもよい
        '                "sampling_rate":90000'
        "            },"
        '            "audio_params":{'
        '                "band_width":1500,'
        '                "codec":"OPUS",'
        '                "payload_type":111,'  # ここで指定するpayload_typeはgStreamerで指定するものと一致させること。番号はRFCに沿っていれば何番でもよい
        '                "sampling_rate":48000'
        "            }"
        "        },"
        '        "redirect_params":{'
        '            "video":{'
        '               "ip_v4":"127.0.0.1",'
        '               "port":%d'
        "            },"
        '            "video_rtcp":{'
        '                "ip_v4":"127.0.0.1",'
        '                "port":%d'
        "            },"
        '            "audio":{'
        '                "ip_v4":"127.0.0.1",'
        '               "port":%d'
        "            },"
        '            "audio_rtcp":{'
        '                "ip_v4":"127.0.0.1",'
        '               "port":%d'
        "            }"
        "        }"
        "    }"
        "}"
        % (
            peer_id,
            token,
            target_id,
            video_port,
            video_rtcp_port,
            audio_port,
            audio_rtcp_port,
        )
    )
    return answer_query


def create_answer_query(media_connection_id):
    video_port = find_free_port()
    video_rtcp_port = find_free_port()
    audio_port = find_free_port()
    audio_rtcp_port = find_free_port()

    answer_query = (
        "{"
        '    "request_type":"MEDIA",'
        '    "command":"ANSWER",'
        '    "params":{'
        '        "media_connection_id":"%s",'
        '        "answer_query":{'
        '            "constraints":{'
        '                "video_params":{'
        '                    "band_width":1500,'
        '                    "codec":"H264",'
        '                    "payload_type":96,'  # ここで指定するpayload_typeはgStreamerで指定するものと一致させること。番号はRFCに沿っていれば何番でもよい
        '                    "sampling_rate":90000'
        "                },"
        '                "audio_params":{'
        '                    "band_width":1500,'
        '                    "codec":"OPUS",'
        '                    "payload_type":111,'  # ここで指定するpayload_typeはgStreamerで指定するものと一致させること。番号はRFCに沿っていれば何番でもよい
        '                    "sampling_rate":48000'
        "                }"
        "            },"
        '            "redirect_params":{'
        '                "video":{'
        '                    "ip_v4":"127.0.0.1",'
        '                    "port":%d'
        "                },"
        '                "video_rtcp":{'
        '                    "ip_v4":"127.0.0.1",'
        '                    "port":%d'
        "                },"
        '                "audio":{'
        '                    "ip_v4":"127.0.0.1",'
        '                    "port":%d'
        "                },"
        '                "audio_rtcp":{'
        '                    "ip_v4":"127.0.0.1",'
        '                    "port":%d'
        "                }"
        "            }"
        "        }"
        "    }"
        "}"
        % (
            media_connection_id,
            video_port,
            video_rtcp_port,
            audio_port,
            audio_rtcp_port,
        )
    )
    return answer_query


def create_media_status_request(media_connection_id):
    return (
        "{"
        '    "request_type":"MEDIA",'
        '    "command":"STATUS",'
        '    "params":{'
        '       "media_connection_id":"%s"'
        "    }"
        "}" % media_connection_id
    )


def parse_media_info(object):
    return (
        # RTPの送信先IPアドレス
        object["ip_v4"],
        # RTPの送信先ポート
        object["port"],
        # RTCPの送信先IPアドレス
        object["ip_v4"],
        # RTCPの送信先ポート
        object["port"],
    )
