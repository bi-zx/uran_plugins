from types import SimpleNamespace

import pytest

from uran_autotask_plugin.mission.mission_manager import MissionManager


def recording_manager(config):
    manager = MissionManager.__new__(MissionManager)
    manager._media_actions_cfg = config
    manager._task = SimpleNamespace(media_record=True)
    manager._recording_active = False
    return manager


def test_recording_uses_the_selected_product_channel():
    manager = recording_manager({"record_channel_id": "inspection_camera"})
    published = []
    manager._publish_media_control = lambda **packet: published.append(packet)
    manager._start_recording_if_needed()
    assert manager._recording_active
    manager._stop_recording_if_needed()
    assert not manager._recording_active
    assert published == [
        {"action": "record_start", "channel_id": "inspection_camera"},
        {"action": "record_stop", "channel_id": "inspection_camera"}]


def test_recording_without_channel_configuration_never_selects_a_camera():
    manager = recording_manager({})
    published = []
    manager._publish_media_control = lambda **packet: published.append(packet)
    with pytest.raises(ValueError, match="record_channel_id"):
        manager._start_recording_if_needed()
    assert not published and not manager._recording_active


def test_failed_recording_request_does_not_report_recording_active():
    manager = recording_manager({"record_channel_id": "inspection_camera"})
    def unavailable(**_packet):
        raise RuntimeError("media worker is unavailable")
    manager._publish_media_control = unavailable
    with pytest.raises(RuntimeError, match="unavailable"):
        manager._start_recording_if_needed()
    assert not manager._recording_active
