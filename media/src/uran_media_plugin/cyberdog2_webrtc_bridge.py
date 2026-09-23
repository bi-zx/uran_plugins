"""cyberdog2_webrtc_bridge.py — CyberDog2 WebRTC 信令桥接工具

不使用 aiortc，而是将信令中继给 image_transmission 节点处理。
image_transmission 负责实际的 WebRTC 推流，uran_media 只做信令格式转换与转发。

image_transmission 信令 topic：
  img_trans_signal_in  (std_msgs/String) — 接收来自 app/gRPC 的信令
  img_trans_signal_out (std_msgs/String) — 发出应答信令给 app/gRPC

信令 JSON 格式（image_transmission 侧）：
  SDP offer:     {"uid": "<uid>", "offer_sdp":  {"type": "offer",  "sdp": "..."}}
  SDP answer:    {"uid": "<uid>", "answer_sdp": {"type": "answer", "sdp": "..."}}
  ICE candidate: {"uid": "<uid>", "c_sdp": {"sdpMid": "...", "sdpMLineIndex": 0, "candidate": "..."}}
  Stop:          {"uid": "<uid>", "is_closed": true}
  Error:         {"uid": "<uid>", "error": {"code": 1001, "msg": "..."}}
"""

import json


def to_img_trans(uid: str, signal: dict) -> str:
    """将 URAN 信令格式转换为 image_transmission JSON 字符串。

    URAN 信令格式：
      {"type": "offer",     "sdp": "..."}
      {"type": "answer",    "sdp": "..."}
      {"type": "candidate", "sdpMid": "...", "sdpMLineIndex": 0, "candidate": "..."}
    """
    sig_type = signal.get('type', '')

    if sig_type == 'offer':
        return json.dumps({
            'uid': uid,
            'offer_sdp': {'type': 'offer', 'sdp': signal.get('sdp', '')},
        })
    elif sig_type == 'answer':
        return json.dumps({
            'uid': uid,
            'answer_sdp': {'type': 'answer', 'sdp': signal.get('sdp', '')},
        })
    elif sig_type == 'candidate':
        return json.dumps({
            'uid': uid,
            'c_sdp': {
                'sdpMid': signal.get('sdpMid', ''),
                'sdpMLineIndex': signal.get('sdpMLineIndex', 0),
                'candidate': signal.get('candidate', ''),
            },
        })
    return ''


def from_img_trans(raw: str):
    """解析 image_transmission 信令，返回 (uid, signal_dict)。

    signal_dict type 字段：'answer' | 'offer' | 'candidate' | 'closed' | 'error'
    解析失败返回 (None, None)。
    """
    try:
        data = json.loads(raw)
    except Exception:
        return None, None

    uid = data.get('uid', '')

    if 'answer_sdp' in data:
        sdp = data['answer_sdp']
        return uid, {'type': 'answer', 'sdp': sdp.get('sdp', '')}
    elif 'offer_sdp' in data:
        sdp = data['offer_sdp']
        return uid, {'type': 'offer', 'sdp': sdp.get('sdp', '')}
    elif 'c_sdp' in data:
        c = data['c_sdp']
        return uid, {
            'type': 'candidate',
            'sdpMid': c.get('sdpMid', ''),
            'sdpMLineIndex': c.get('sdpMLineIndex', 0),
            'candidate': c.get('candidate', ''),
        }
    elif data.get('is_closed'):
        return uid, {'type': 'closed'}
    elif 'error' in data:
        return uid, {'type': 'error', 'error': data.get('error', {})}

    return uid, None


def make_stop(uid: str) -> str:
    """生成停止信令 JSON。"""
    return json.dumps({'uid': uid, 'is_closed': True})
