import pytest

from uran_mqtt_plugin.client import MqttClient
from uran_mqtt_plugin.worker import MqttWorker


@pytest.mark.parametrize("device_id", [None, "", "   "])
def test_transport_rejects_missing_device_identity(device_id):
    with pytest.raises(ValueError, match="local device identity"):
        MqttClient({"device_id": device_id}, lambda _packet: None)


def test_transport_inherits_core_identity_without_inventing_a_template():
    worker = MqttWorker()
    worker.identity = {"device_id": "cart_027"}
    worker.configure({"transport_enabled": True, "tenant_id": "warehouse"})
    assert worker.default_device_id == worker.client._device_id == "cart_027"
    assert worker.client._downlink_topic == "/oivs/warehouse/cart_027/down"
    packets = []
    worker.client.publish_raw = lambda packet: packets.append(packet) or True
    worker.client._publish_registration()
    assert packets[0]["device_id"] == "cart_027" and packets[0]["template_id"] == ""
    assert not worker.client._is_register_response({"result": "registered", "template_id": ""}, "")


def test_explicit_template_remains_a_profile_choice():
    client = MqttClient({"device_id": "cart_027", "template_id": "warehouse-cart"}, lambda _packet: None)
    assert client._is_register_response({"result": "registered", "template_id": "warehouse-cart"}, "")


def test_transport_config_cannot_override_core_device_identity():
    worker = MqttWorker()
    worker.identity = {"device_id": "cart_027"}
    with pytest.raises(ValueError, match="Core identity"):
        worker.configure({"transport_enabled": True, "device_id": "another_robot"})
