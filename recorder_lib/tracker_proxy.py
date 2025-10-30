"""
Tracker proxy that isolates Vive/SteamVR SDK calls in a separate process.

Main process interacts with a lightweight TrackerClient that exposes get_T().
This avoids segfaults in the main recorder process if the vendor SDK crashes.
"""

import multiprocessing as mp
import time
from typing import Optional
import numpy as np


class TrackerClient:
    """
    Lightweight client that reads poses from a multiprocessing.Queue.
    Provides a get_T() method compatible with ViveToXArmMapper expectations.
    """
    def __init__(self, serial: str, queue: mp.Queue):
        self._serial = serial
        self._queue = queue
        self._last_pose: Optional[np.ndarray] = None
        self._last_update = 0.0

    def get_serial(self) -> str:
        return self._serial

    def get_T(self) -> Optional[np.ndarray]:
        """
        Non-blocking fetch of latest 4x4 pose matrix from proxy process.
        Returns the last cached pose if no newer value is available.
        """
        try:
            while True:
                pose_list = self._queue.get_nowait()
                # Pose is sent as a flat list of 16 floats
                self._last_pose = np.array(pose_list, dtype=np.float32).reshape(4, 4)
                self._last_update = time.time()
        except Exception:
            # Empty queue
            pass
        return self._last_pose


def _tracker_process(serial: Optional[str], queue: mp.Queue, poll_hz: float = 90.0):
    """
    Child process: initializes Vive tracker SDK and pushes poses to queue.
    """
    import sys
    import time
    from vive_teleop_xarm import ViveTrackerModule

    try:
        vive = ViveTrackerModule()
        # Do not call print_discovered_objects to minimize SDK chatter
        trackers = vive.return_selected_devices("tracker")
        tracker_obj = None
        
        # Find tracker by serial number (trackers dict is keyed by device name, not serial)
        if serial and trackers:
            for tracker in trackers.values():
                if tracker.get_serial() == serial:
                    tracker_obj = tracker
                    break
        
        # Fallback to first tracker if serial not found
        if tracker_obj is None and trackers:
            tracker_obj = list(trackers.values())[0]
        
        if tracker_obj is None:
            print("❌ No Vive trackers found in tracker process", file=sys.stderr)
            return

        dt = 1.0 / max(1.0, poll_hz)
        while True:
            try:
                T = tracker_obj.get_T()
                if T is not None:
                    queue.put([float(v) for v in T.flatten()])
                    # Avoid queue growth
                    while queue.qsize() > 1:
                        queue.get_nowait()
            except Exception:
                pass
            time.sleep(dt)
    except Exception as e:
        print(f"❌ Tracker process error: {e}", file=sys.stderr)


def start_tracker_proxy(serial: Optional[str]) -> TrackerClient:
    """
    Spawn a tracker process and return a TrackerClient connected to it.
    """
    queue: mp.Queue = mp.Queue(maxsize=2)
    proc = mp.Process(target=_tracker_process, args=(serial, queue), daemon=True)
    proc.start()
    return TrackerClient(serial or "unknown", queue)


