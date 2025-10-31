#!/usr/bin/env python3
"""
Simple loader for teleoperation recordings.

Usage:
    from load_recording import load_trial
    
    data = load_trial('20250130_143022')
    
    # Access xArm data
    left_joints = data['xarm_left']['joint_positions']
    left_times = data['xarm_left']['timestamps_mono']
    
    # Access Inspire data
    left_hand = data['inspire_left']['angles']
    
    # Metadata
    subject = data['metadata']['subject']
    task = data['metadata']['task']
"""

import sys
from pathlib import Path
from typing import Dict, Any
import numpy as np

try:
    import h5py
except ImportError:
    print("❌ h5py not installed. Install with: pip install h5py")
    sys.exit(1)


def load_trial(trial_id: str, data_dir: str = '/home/joshua/Research/dex-teleop/data') -> Dict[str, Any]:
    """
    Load a recorded trial from HDF5.
    
    Args:
        trial_id: Trial identifier (timestamp format: 20250130_143022)
        data_dir: Directory containing trial files
        
    Returns:
        Dict with structure:
        {
            'metadata': dict of metadata attributes,
            'xarm_left': {
                'timestamps_mono': ndarray,
                'joint_positions': ndarray (N, 7),
                'tcp_poses': ndarray (N, 6),
            },
            'xarm_right': {...},
            'inspire_left': {
                'timestamps_mono': ndarray,
                'angles': ndarray (N, 6),
            },
            'inspire_right': {...},
        }
    """
    trial_file = Path(data_dir) / f"trial_{trial_id}.h5"
    
    if not trial_file.exists():
        raise FileNotFoundError(f"Trial not found: {trial_file}")
    
    data = {}
    
    with h5py.File(trial_file, 'r') as f:
        # Load metadata
        data['metadata'] = dict(f['metadata'].attrs)
        
        # Load xArm data
        for arm_label in ['left', 'right']:
            device = f'xarm_{arm_label}'
            if device in f:
                data[device] = {
                    'timestamps_mono': f[device]['timestamps_mono'][:],
                    'joint_positions': f[device]['joint_positions'][:],
                    'tcp_poses': f[device]['tcp_poses'][:],
                }
        
        # Load Inspire data
        for hand_label in ['left', 'right']:
            device = f'inspire_{hand_label}'
            if device in f:
                data[device] = {
                    'timestamps_mono': f[device]['timestamps_mono'][:],
                    'angles': f[device]['angles'][:],
                }
    
    return data


def print_trial_summary(data: Dict[str, Any]):
    """Print a summary of loaded trial data."""
    meta = data['metadata']
    
    print("="*60)
    print(f"Trial Summary")
    print("="*60)
    print(f"Subject: {meta.get('subject', 'unknown')}")
    print(f"Task: {meta.get('task', 'untitled')}")
    print(f"Duration: {meta.get('duration_s', 0.0):.1f}s")
    print(f"Timestamp: {meta.get('timestamp', 'unknown')}")
    print()
    
    # xArm data
    for device in ['xarm_left', 'xarm_right']:
        if device in data:
            n_samples = len(data[device]['timestamps_mono'])
            duration = data[device]['timestamps_mono'][-1] - data[device]['timestamps_mono'][0]
            rate = n_samples / duration if duration > 0 else 0
            print(f"{device}:")
            print(f"  Samples: {n_samples}")
            print(f"  Duration: {duration:.2f}s")
            print(f"  Rate: {rate:.1f} Hz")
    
    # Inspire data
    for device in ['inspire_left', 'inspire_right']:
        if device in data:
            n_samples = len(data[device]['timestamps_mono'])
            duration = data[device]['timestamps_mono'][-1] - data[device]['timestamps_mono'][0]
            rate = n_samples / duration if duration > 0 else 0
            print(f"{device}:")
            print(f"  Samples: {n_samples}")
            print(f"  Duration: {duration:.2f}s")
            print(f"  Rate: {rate:.1f} Hz")
    
    print("="*60)


if __name__ == '__main__':
    """Example usage: load and print summary."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Load and inspect a recording")
    parser.add_argument('trial_id', help='Trial ID to load')
    parser.add_argument('--data-dir', default='/home/joshua/Research/dex-teleop/data',
                       help='Data directory')
    args = parser.parse_args()
    
    print(f"Loading trial: {args.trial_id}")
    data = load_trial(args.trial_id, data_dir=args.data_dir)
    print_trial_summary(data)
    
    # Example: access data
    print("\nExample data access:")
    if 'xarm_left' in data:
        print(f"  Left arm first joint position: {data['xarm_left']['joint_positions'][0]}")
    if 'inspire_left' in data:
        print(f"  Left hand first angles: {data['inspire_left']['angles'][0]}")






