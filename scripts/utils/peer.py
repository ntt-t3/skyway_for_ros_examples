# -*- coding: utf-8 -*-


def create_peer_request(peer_id, api_key):
    return (
        "{"
        '  "request_type":"PEER",'
        '  "command":"CREATE",'
        '  "params":{'
        '      "key":"%s",'
        '      "domain":"localhost",'
        '      "peer_id":"%s",'
        '      "turn":false'
        "  }"
        "}" % (api_key, peer_id)
    )


def delete_peer_request(peer_id, token):
    return (
        "{"
        '   "request_type": "PEER", '
        '   "command": "DELETE", '
        '   "params": {'
        '       "peer_id": "%s",'
        '       "token": "%s"'
        "   }"
        "}" % (peer_id, token)
    )


def create_peer_status_request(peer_id, token):
    return (
        "{"
        '   "request_type": "PEER",'
        '   "command": "STATUS",'
        '   "params": {'
        '       "peer_id": "%s", '
        '       "token": "%s"'
        "  }"
        "}" % (peer_id, token)
    )
