from typing import Any, Dict, Optional


class ClosedLoopManager:
    def __init__(self, *, gps_supervisor=None, visual_pose_supervisor=None):
        self._gps_supervisor = gps_supervisor
        self._visual_pose_supervisor = visual_pose_supervisor

    def reset(self):
        if self._gps_supervisor is not None:
            self._gps_supervisor.reset()
        if self._visual_pose_supervisor is not None:
            self._visual_pose_supervisor.reset()

    def evaluate(
        self,
        *,
        projector,
        pose_registry,
        now_ns: int,
        monotonic_s: float,
    ) -> Optional[Dict[str, Any]]:
        if self._gps_supervisor is None:
            gps_result = None
        else:
            gps_result = self._gps_supervisor.evaluate(
                projector=projector,
                pose_registry=pose_registry,
                now_ns=now_ns,
                monotonic_s=monotonic_s,
            )
        if gps_result is not None:
            return gps_result
        if self._visual_pose_supervisor is None:
            return None
        return self._visual_pose_supervisor.evaluate(
            pose_registry=pose_registry,
            now_ns=now_ns,
            monotonic_s=monotonic_s,
        )

    def gps_state_snapshot(self) -> Dict[str, Any]:
        if self._gps_supervisor is None:
            return {}
        return self._gps_supervisor.state_snapshot()

    def state_snapshot(self) -> Dict[str, Any]:
        return {
            'gps': self.gps_state_snapshot(),
            'visual_odom': (
                self._visual_pose_supervisor.state_snapshot()
                if self._visual_pose_supervisor is not None else {}
            ),
        }
