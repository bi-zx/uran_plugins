from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple


class LeaseConflict(RuntimeError):
    pass


@dataclass(frozen=True)
class Lease:
    lease_id: str
    device_id: str
    component_id: str
    resource_group: str
    owner: str
    acquired_at_ns: int
    expires_at_ns: int
    reason: str = ""
    priority: int = 0
    task_id: str = ""

    def to_dict(self):
        result = {
            "schema_version": "1.0",
            "lease_id": self.lease_id,
            "resource": {
                "device_id": self.device_id,
                "component_id": self.component_id,
                "resource_group": self.resource_group,
            },
            "owner": self.owner,
            "priority": self.priority,
            "acquired_at_ns": self.acquired_at_ns,
            "expires_at_ns": self.expires_at_ns,
        }
        if self.reason:
            result["reason"] = self.reason
        if self.task_id:
            result["task_id"] = self.task_id
        return result

    @classmethod
    def from_contract(cls, document):
        resource = document["resource"]
        return cls(
            lease_id=str(document["lease_id"]),
            device_id=str(resource["device_id"]),
            component_id=str(resource["component_id"]),
            resource_group=str(resource["resource_group"]),
            owner=str(document["owner"]),
            acquired_at_ns=int(document["acquired_at_ns"]),
            expires_at_ns=int(document["expires_at_ns"]),
            reason=str(document.get("reason") or ""),
            priority=int(document["priority"]),
            task_id=str(document.get("task_id") or ""),
        )


class ControlArbiter:
    """Arbitrates stable component/resource identities, never plugin instances."""

    def __init__(self):
        self._leases: Dict[Tuple[str, str], Lease] = {}

    @staticmethod
    def _key(device_id: str, resource_group: str) -> Tuple[str, str]:
        if not device_id or not resource_group:
            raise ValueError("device_id and resource_group are required")
        return device_id, resource_group

    def acquire(self, lease: Lease, *, now_ns: int) -> Optional[Lease]:
        if lease.acquired_at_ns > now_ns or lease.expires_at_ns <= now_ns:
            raise ValueError("lease interval is not active")
        key = self._key(lease.device_id, lease.resource_group)
        current = self._leases.get(key)
        if current and current.expires_at_ns > now_ns:
            if current.lease_id == lease.lease_id and current.owner == lease.owner:
                self._leases[key] = lease
                return None
            if lease.priority > current.priority:
                self._leases[key] = lease
                return current
            raise LeaseConflict(
                f"resource {lease.resource_group} is held by {current.owner}")
        self._leases[key] = lease
        return None

    def release(self, *, lease_id: str, device_id: str, resource_group: str,
                owner: str = "", now_ns: int) -> bool:
        key = self._key(device_id, resource_group)
        current = self._leases.get(key)
        if current is None or current.expires_at_ns <= now_ns:
            self._leases.pop(key, None)
            return False
        if current.lease_id != lease_id or (owner and current.owner != owner):
            raise LeaseConflict("lease identity or owner does not match")
        del self._leases[key]
        return True

    def permits(self, *, lease_id: str, device_id: str, component_id: str,
                resource_group: str, owner: str, now_ns: int) -> bool:
        key = self._key(device_id, resource_group)
        current = self._leases.get(key)
        if current is None or current.expires_at_ns <= now_ns:
            self._leases.pop(key, None)
            return False
        return (current.lease_id == lease_id and current.owner == owner and
                current.component_id == component_id)

    def expire(self, *, now_ns: int) -> Iterable[Lease]:
        expired = [lease for lease in self._leases.values()
                   if lease.expires_at_ns <= now_ns]
        for lease in expired:
            self._leases.pop((lease.device_id, lease.resource_group), None)
        return expired

    def get(self, device_id: str, resource_group: str, *, now_ns: int) -> Optional[Lease]:
        self.expire(now_ns=now_ns)
        return self._leases.get(self._key(device_id, resource_group))

    def snapshot(self, *, now_ns: int):
        self.expire(now_ns=now_ns)
        return [lease.to_dict() for lease in self._leases.values()]
