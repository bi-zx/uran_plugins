import pytest

from uran_control_arbiter_plugin import ControlArbiter, Lease, LeaseConflict


def _lease(lease_id, component, group, owner="task", expires=200):
    return Lease(lease_id, "robot", component, group, owner, 100, expires)


def test_independent_arm_resources_do_not_block_each_other():
    arbiter = ControlArbiter()
    arbiter.acquire(_lease("left", "arm_left", "arm_left"), now_ns=100)
    arbiter.acquire(_lease("right", "arm_right", "arm_right"), now_ns=100)
    assert len(arbiter.snapshot(now_ns=100)) == 2


def test_same_resource_group_conflict_is_rejected():
    arbiter = ControlArbiter()
    arbiter.acquire(_lease("a", "base", "locomotion"), now_ns=100)
    with pytest.raises(LeaseConflict):
        arbiter.acquire(_lease("b", "base", "locomotion", owner="cloud"), now_ns=100)


def test_expired_lease_can_be_replaced():
    arbiter = ControlArbiter()
    arbiter.acquire(_lease("a", "base", "locomotion", expires=150), now_ns=100)
    replacement = _lease("b", "base", "locomotion", owner="cloud", expires=300)
    assert arbiter.acquire(replacement, now_ns=200) is None
    assert arbiter.get("robot", "locomotion", now_ns=200) == replacement


def test_higher_priority_lease_preempts_and_returns_previous_owner():
    arbiter = ControlArbiter()
    low = Lease("a", "robot", "base", "locomotion", "task", 100, 300,
                priority=1)
    high = Lease("b", "robot", "base", "locomotion", "operator", 110, 300,
                 priority=10)
    arbiter.acquire(low, now_ns=110)
    assert arbiter.acquire(high, now_ns=110) == low
    assert arbiter.get("robot", "locomotion", now_ns=110) == high


def test_contract_shape_round_trips_without_flattening_resource():
    lease = Lease("a", "robot", "arm_left", "arm-motion", "task", 100, 300,
                  reason="mission", priority=4, task_id="task-1")
    assert Lease.from_contract(lease.to_dict()) == lease
