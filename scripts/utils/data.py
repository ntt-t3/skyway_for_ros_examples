# -*- coding: utf-8 -*-

import json


def create_connect_request(
    peer_id,
    token,
    target_id,
    plugin_info,
    metadata
):
    message = json.loads(
        "{"
        '   "request_type":"DATA",'
        '   "command":"CONNECT",'
        '   "params": {'
        '      "peer_id": "%s", '
        '      "token": "%s", '
        '      "target_id": "%s", '
        '      "plugin_info": %s, '
        '       "options": {'
        '           "serialization": "NONE", '
        '           "dcInit": {'
        '               "ordered": false, '  # パケットの順番をソートさせない場合
        '               "maxRetransmits": 0 '  # リアルタイム通信を阻害しないよう再送させない場合
        "           }"
        "       }"
        "   }"
        "}" % (peer_id, token, target_id, json.dumps(plugin_info))
    )

    message["params"]["options"]["metadata"] = json.dumps(metadata)
    return json.dumps(message)


def redirect_request(data_connection_id, plugin_info):
    return (
        "{"
        '   "request_type":"DATA",'
        '   "command":"REDIRECT",'
        '   "params":{'
        '     "data_connection_id": "%s",'
        '     "plugin_info": %s'
        "   }"
        "}" % (data_connection_id, json.dumps(plugin_info))
    )
