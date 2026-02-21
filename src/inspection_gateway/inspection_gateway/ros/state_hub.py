import queue
import threading
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class StateSnapshot:
    ros_state: object


class _LatestQueue:
    def __init__(self) -> None:
        self._q: queue.Queue = queue.Queue(maxsize=1)

    def put_latest(self, item: object) -> None:
        try:
            self._q.put_nowait(item)
        except queue.Full:
            try:
                _ = self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(item)
            except queue.Full:
                pass

    def get(self, timeout_s: float) -> object:
        return self._q.get(timeout=timeout_s)


class StateHub:
    """
    Thread-safe hub for SystemState snapshots.

    ROS subscription thread calls publish().
    WebSocket / HTTP handlers call subscribe() and wait on per-subscriber queues.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._subs: dict[int, _LatestQueue] = {}
        self._next_id = 1
        self._last: Optional[StateSnapshot] = None

    def publish(self, ros_state: object) -> None:
        with self._lock:
            self._last = StateSnapshot(ros_state=ros_state)
            subs = list(self._subs.values())
        for q in subs:
            q.put_latest(ros_state)

    def get_last(self) -> Optional[StateSnapshot]:
        with self._lock:
            return self._last

    def subscribe(self) -> tuple[int, _LatestQueue]:
        with self._lock:
            sub_id = self._next_id
            self._next_id += 1
            q = _LatestQueue()
            self._subs[sub_id] = q
            return sub_id, q

    def unsubscribe(self, sub_id: int) -> None:
        with self._lock:
            self._subs.pop(sub_id, None)

