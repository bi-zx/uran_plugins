"""T1.3 MQTT 客户端 — 入网、注册、心跳"""
import json
import logging
import os
import threading
import time
from typing import Callable, Dict, Optional

logger = logging.getLogger(__name__)

try:
    import paho.mqtt.client as mqtt
    _PAHO_OK = True
except ImportError:
    _PAHO_OK = False
    logger.warning('paho-mqtt not installed; MQTT disabled')


class MqttClient:
    """封装 paho-mqtt，提供连接、注册、发布、订阅能力。"""

    def __init__(self, net_cfg: dict, downlink_cb: Callable[[dict], None],
                 connection_cb: Optional[Callable[[bool, int], None]] = None):
        self._net_cfg = net_cfg
        self._downlink_cb = downlink_cb
        self._connection_cb = connection_cb
        self._client: Optional['mqtt.Client'] = None
        self._lock = threading.Lock()
        self._connect_lock = threading.Lock()
        self._connected = False
        self._registered = False
        self._reg_event = threading.Event()
        self._reg_result = 'timeout'
        self._registration_sent = False

        device_id = net_cfg.get('device_id', 'device_001')
        tenant_id = net_cfg.get('tenant_id', 'default')
        mqtt_cfg = net_cfg.get('mqtt', {})
        registration_cfg = net_cfg.get('registration', {})
        prefix = mqtt_cfg.get('topic_prefix', '/oivs/{tenant_id}/{device_id}')
        prefix = prefix.replace('{tenant_id}', tenant_id).replace('{device_id}', device_id)
        prefix = prefix.replace('{tenant}', tenant_id)  # legacy placeholder

        self._device_id = device_id
        self._template_id = net_cfg.get('template_id', 'template_001')
        auth_cfg = net_cfg.get('auth', {})
        configured_username = auth_cfg.get('username', '')
        username_env = auth_cfg.get('username_env', '')
        if username_env:
            configured_username = os.environ.get(str(username_env), configured_username)
        # Keep compatibility with deployments that used device_id as the
        # MQTT username before auth.username was introduced.
        if configured_username in ('', 'changeme', None):
            configured_username = self._device_id
        self._mqtt_username = configured_username
        token = auth_cfg.get('token', '')
        token_env = auth_cfg.get('token_env', '')
        if token_env:
            token = os.environ.get(str(token_env), token)
        self._token = token
        self._broker_host = mqtt_cfg.get('broker_host', 'localhost')
        self._broker_port = int(mqtt_cfg.get('broker_port', 1883))
        self._keepalive = int(mqtt_cfg.get('keepalive', 60))
        self._uplink_topic = f'{prefix}/up'
        self._downlink_topic = f'{prefix}/down'
        self._require_register_response = bool(registration_cfg.get('require_response', False))
        self._legacy_online_on_timeout = bool(
            registration_cfg.get('legacy_online_on_timeout', not self._require_register_response)
        )

        # 通路状态
        self._latency_ms: int = -1
        self._last_check_ts: int = 0

    # ------------------------------------------------------------------ connect
    def connect(self) -> bool:
        if not _PAHO_OK:
            return False
        with self._connect_lock:
            with self._lock:
                if self._connected and self._client is not None:
                    return True
                old_client = self._client
                self._client = None
                self._connected = False
                self._registered = False
                self._registration_sent = False
                self._reg_result = 'timeout'
                self._reg_event.clear()

            if old_client is not None:
                try:
                    old_client.loop_stop()
                    old_client.disconnect()
                except Exception:
                    pass

            client = mqtt.Client(client_id=self._device_id)
            if self._token:
                client.username_pw_set(self._mqtt_username, self._token)
            client.on_connect = self._on_connect
            client.on_disconnect = self._on_disconnect
            client.on_message = self._on_message
            with self._lock:
                self._client = client

            try:
                client.connect(self._broker_host, self._broker_port, self._keepalive)
                client.loop_start()
                # 等待连接最多 5 秒，失败交给上层定时器重试。
                for _ in range(50):
                    if self.is_connected():
                        return True
                    time.sleep(0.1)
                logger.error(
                    f'MQTT connect timeout: {self._broker_host}:{self._broker_port}'
                )
            except Exception as exc:
                logger.error(
                    f'MQTT connect error ({self._broker_host}:{self._broker_port}): {exc}'
                )

            try:
                client.loop_stop()
                client.disconnect()
            except Exception:
                pass
            with self._lock:
                if self._client is client:
                    self._client = None
                self._connected = False
                self._registered = False
            return False

    def disconnect(self):
        with self._lock:
            client = self._client
            self._client = None
            self._connected = False
            self._registered = False
        if client:
            client.loop_stop()
            client.disconnect()

    # ------------------------------------------------------------------ registration
    def register(self, timeout_s: float = 10.0) -> str:
        """发送注册请求，返回 'registered'/'auto_registered'/'rejected'/'timeout'。"""
        if not self._registration_sent and not self._publish_registration():
            logger.error(
                f'MQTT register publish failed, topic={self._uplink_topic}'
            )
        self._reg_event.wait(timeout=timeout_s)
        with self._lock:
            connected = self._connected
        if (
            self._reg_result == 'timeout'
            and connected
            and self._legacy_online_on_timeout
        ):
            self._reg_result = 'legacy_connected'
            with self._lock:
                self._registered = True
        return self._reg_result

    def _publish_registration(self) -> bool:
        payload = {
            'msg_type': 'register',
            'msg_version': '1.0',
            'device_id': self._device_id,
            'template_id': self._template_id,
            'token': self._token,
            'timestamp_ms': int(time.time() * 1000),
        }
        published = self.publish_raw(payload)
        with self._lock:
            self._registration_sent = published
        return published

    # ------------------------------------------------------------------ publish
    def publish_uplink(self, payload: dict, qos: int = 1) -> bool:
        return self.publish_raw(payload, qos=qos)

    def publish_raw(self, payload: dict, qos: int = 1) -> bool:
        with self._lock:
            if not self._connected or self._client is None:
                return False
            client = self._client
        try:
            data = json.dumps(payload)
            result = client.publish(self._uplink_topic, data, qos=qos)
            if result.rc != 0:
                logger.error(f'MQTT publish rejected rc={result.rc}, topic={self._uplink_topic}')
                return False
            return True
        except Exception as exc:
            logger.error(f'MQTT publish error: {exc}')
            return False

    # ------------------------------------------------------------------ status
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def is_registered(self) -> bool:
        with self._lock:
            return self._registered

    def get_protocol_entry(self) -> dict:
        return {
            'available': self.is_connected(),
            'latency_ms': self._latency_ms,
            'last_check_ts': self._last_check_ts,
        }

    # ------------------------------------------------------------------ paho callbacks
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            with self._lock:
                if self._client is not client:
                    return
                self._connected = True
                self._registered = False
            self._last_check_ts = int(time.time())
            try:
                result = client.subscribe(self._downlink_topic, qos=1)
                subscribe_rc = result[0]
            except Exception as exc:
                subscribe_rc = None
                logger.error(
                    f'MQTT subscribe error, topic={self._downlink_topic}: {exc}'
                )
            if subscribe_rc != mqtt.MQTT_ERR_SUCCESS:
                with self._lock:
                    if self._client is client:
                        self._connected = False
                        self._registered = False
                logger.error(
                    f'MQTT subscribe rejected rc={subscribe_rc}, topic={self._downlink_topic}'
                )
                return
            with self._lock:
                self._registered = False
                self._registration_sent = False
                self._reg_result = 'timeout'
                self._reg_event.clear()
            if not self._publish_registration():
                logger.error(f'MQTT registration publish failed, topic={self._uplink_topic}')
            if self._connection_cb:
                try:
                    self._connection_cb(True, rc)
                except Exception as exc:
                    logger.error(f'MQTT connection callback error: {exc}')
            logger.info(f'MQTT connected → subscribed {self._downlink_topic}')
        else:
            logger.error(f'MQTT connect refused rc={rc}')

    def _on_disconnect(self, client, userdata, rc):
        with self._lock:
            if self._client is not client:
                return
            self._connected = False
            self._registered = False
            self._registration_sent = False
        self._last_check_ts = int(time.time())
        if self._connection_cb:
            try:
                self._connection_cb(False, rc)
            except Exception as exc:
                logger.error(f'MQTT disconnection callback error: {exc}')
        logger.warning(f'MQTT disconnected rc={rc}')

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except Exception as exc:
            logger.error(f'MQTT message decode error: {exc}')
            return

        msg_type = payload.get('msg_type', '')

        # 处理注册响应
        if self._is_register_response(payload, msg_type):
            self._reg_result = payload.get('result', 'rejected')
            with self._lock:
                self._registered = self._reg_result in ('registered', 'auto_registered')
            self._reg_event.set()
            return

        # 其余消息交给节点处理
        try:
            self._downlink_cb(payload)
        except Exception as exc:
            logger.error(f'downlink callback error: {exc}')

    def _is_register_response(self, payload: Dict, msg_type: str) -> bool:
        """Accept both the current and legacy registration response shapes."""
        if msg_type == 'register_response':
            return True
        if msg_type:
            return False

        result = payload.get('result')
        if result not in ('registered', 'auto_registered', 'rejected'):
            return False

        device_id = payload.get('device_id')
        template_id = payload.get('template_id')
        return device_id == self._device_id or template_id == self._template_id
