import json
import socketserver


def find_free_port():
    with socketserver.TCPServer(("localhost", 0), None) as s:
        return s.server_address[1]


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
        f'        "media_connection_id":"{media_connection_id}",'
        '        "answer_query":{'
        '            "constraints":{'
        '                "video_params":{'
        '                    "band_width":1500,'
        '                    "codec":"H264",'
        '                    "payload_type":96,'
        '                    "sampling_rate":90000'
        "                },"
        '                "audio_params":{'
        '                    "band_width":1500,'
        '                    "codec":"OPUS",'
        '                    "payload_type":111,'
        '                    "sampling_rate":48000'
        "                }"
        "            },"
        '            "redirect_params":{'
        '                "video":{'
        '                    "ip_v4":"127.0.0.1",'
        f'                    "port":{video_port}'
        "                },"
        '                "video_rtcp":{'
        '                    "ip_v4":"127.0.0.1",'
        f'                    "port":{video_rtcp_port}'
        "                },"
        '                "audio":{'
        '                    "ip_v4":"127.0.0.1",'
        f'                    "port":{audio_port}'
        "                },"
        '                "audio_rtcp":{'
        '                    "ip_v4":"127.0.0.1",'
        f'                    "port":{audio_rtcp_port}'
        "                }"
        "            }"
        "        }"
        "    }"
        "}"
    )
    return answer_query
