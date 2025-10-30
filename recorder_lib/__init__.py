"""
Teleoperation recorder library for dual-tracker xArm + Inspire Hands recording.
"""

from .pedal import RecorderPedal
from .xarm_worker import XArmRecorder
from .inspire_worker import InspireRecorder
from .writer import HDF5Writer
from .state_machine import RecorderOrchestrator, RecorderState
from .replay import replay_trial, list_trials, validate_trial
from .tracker_proxy import start_tracker_proxy, TrackerClient
from .xarm_io_proxy import start_xarm_io_proxy, XArmIOClient

__all__ = [
    'RecorderPedal',
    'XArmRecorder',
    'InspireRecorder',
    'HDF5Writer',
    'RecorderOrchestrator',
    'RecorderState',
    'replay_trial',
    'list_trials',
    'validate_trial',
    'start_tracker_proxy',
    'TrackerClient',
    'start_xarm_io_proxy',
    'XArmIOClient',
]


