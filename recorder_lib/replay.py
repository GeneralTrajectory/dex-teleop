"""
Replay functionality for recorded teleoperation trials.
"""

import sys
import time
import os
from pathlib import Path
from typing import Optional
import numpy as np

try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False
    print("⚠️ h5py not installed. Install with: pip install h5py")

# Add xArm SDK to path
sys.path.insert(0, '/home/joshua/Documents/Sources/Papers/Cursor/AnyDexGrasp')


def list_trials(data_dir: str = '/home/joshua/Research/dex-teleop/data'):
    """
    List all available trials.
    
    Args:
        data_dir: Directory containing trial files
    """
    if not HDF5_AVAILABLE:
        print("❌ h5py not installed")
        return
    
    data_path = Path(data_dir)
    if not data_path.exists():
        print(f"❌ Data directory not found: {data_dir}")
        return
    
    trial_files = sorted(data_path.glob('trial_*.h5'))
    
    if not trial_files:
        print(f"No trials found in {data_dir}")
        return
    
    print(f"Found {len(trial_files)} trial(s):\n")
    
    for trial_file in trial_files:
        trial_id = trial_file.stem.replace('trial_', '')
        
        try:
            with h5py.File(trial_file, 'r') as f:
                meta = f['metadata'].attrs
                subject = meta.get('subject', 'unknown')
                task = meta.get('task', 'untitled')
                duration = meta.get('duration_s', 0.0)
                timestamp = meta.get('timestamp', 'unknown')
                active_arms_str = meta.get('active_arms', 'left,right')
                active_arms = active_arms_str.split(',')
                
                # Count samples
                xarm_left_samples = len(f['/xarm_left/timestamps_mono']) if '/xarm_left/timestamps_mono' in f else 0
                xarm_right_samples = len(f['/xarm_right/timestamps_mono']) if '/xarm_right/timestamps_mono' in f else 0
                
                print(f"Trial: {trial_id}")
                print(f"  Subject: {subject}")
                print(f"  Task: {task}")
                print(f"  Timestamp: {timestamp}")
                print(f"  Duration: {duration:.1f}s")
                print(f"  Arms: {', '.join(active_arms)}")
                print(f"  Samples: xArm_L={xarm_left_samples}, xArm_R={xarm_right_samples}")
                print()
        
        except Exception as e:
            print(f"Trial: {trial_id}")
            print(f"  ❌ Error reading file: {e}")
            print()


def replay_trial(trial_id: str, speed: float = 1.0, dry_run: bool = False,
                data_dir: str = '/home/joshua/Research/dex-teleop/data'):
    """
    Replay a recorded trial.
    
    Args:
        trial_id: Trial identifier
        speed: Playback speed multiplier (1.0 = real-time)
        dry_run: If True, don't move robots (just validate and plot)
        data_dir: Directory containing trial files
    """
    if not HDF5_AVAILABLE:
        print("❌ h5py not installed")
        return 1
    
    trial_file = Path(data_dir) / f"trial_{trial_id}.h5"
    if not trial_file.exists():
        print(f"❌ Trial not found: {trial_file}")
        return 1
    
    print("="*60)
    print(f"▶️  Replaying trial: {trial_id}")
    print("="*60)
    
    # Load data
    try:
        with h5py.File(trial_file, 'r') as f:
            # Load metadata
            meta = f['metadata'].attrs
            subject = meta.get('subject', 'unknown')
            task = meta.get('task', 'untitled')
            duration = meta.get('duration_s', 0.0)
            active_arms_str = meta.get('active_arms', 'left,right')
            active_arms = active_arms_str.split(',')
            
            print(f"\nMetadata:")
            print(f"  Subject: {subject}")
            print(f"  Task: {task}")
            print(f"  Duration: {duration:.1f}s")
            print(f"  Arms: {', '.join(active_arms)}")
            print(f"  Playback speed: {speed}x")
            print()
            
            # Load trajectories (only for active arms)
            trajectories = {}
            for arm_label in active_arms:
                device = f'xarm_{arm_label}'
                if device in f and 'timestamps_mono' in f[device]:
                    trajectories[arm_label] = {
                        'timestamps': f[device]['timestamps_mono'][:],
                        'joint_positions': f[device]['joint_positions'][:],
                        'tcp_poses': f[device]['tcp_poses'][:],
                    }
                    n_samples = len(trajectories[arm_label]['timestamps'])
                    print(f"✅ Loaded {arm_label} arm: {n_samples} samples")
            
            # Load Inspire data (for display only, only for active arms)
            for hand_label in active_arms:
                device = f'inspire_{hand_label}'
                if device in f and 'timestamps_mono' in f[device]:
                    n_samples = len(f[device]['timestamps_mono'])
                    print(f"✅ Loaded {hand_label} hand: {n_samples} samples")
    
    except Exception as e:
        print(f"❌ Error loading trial: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    if not trajectories:
        print("❌ No xArm trajectory data found")
        return 1
    
    # Dry-run mode: just validate and show stats
    if dry_run:
        print("\n" + "="*60)
        print("🔍 DRY RUN - No robot motion")
        print("="*60)
        
        for label, traj in trajectories.items():
            times = traj['timestamps']
            joints = traj['joint_positions']
            
            duration = times[-1] - times[0]
            actual_rate = len(times) / duration if duration > 0 else 0
            
            print(f"\n{label.capitalize()} arm:")
            print(f"  Samples: {len(times)}")
            print(f"  Duration: {duration:.2f}s")
            print(f"  Actual rate: {actual_rate:.1f} Hz")
            print(f"  Joint ranges:")
            for i in range(7):
                joint_vals = joints[:, i]
                print(f"    J{i+1}: [{joint_vals.min():.1f}, {joint_vals.max():.1f}]°")
        
        print("\n✅ Dry run complete")
        return 0
    
    # Live replay
    print("\n" + "="*60)
    print("▶️  LIVE REPLAY - Robot will move")
    print("="*60)
    print("⚠️  Ensure workspace is clear")
    print("⚠️  Emergency stop button accessible")
    print()
    
    response = input("Continue with live replay? [y/N]: ")
    if response.lower() != 'y':
        print("Cancelled by user")
        return 0
    
    # Import xArm SDK
    try:
        from xarm.wrapper import XArmAPI
    except ImportError:
        print("❌ xArm SDK not installed")
        return 1
    
    # Connect to arms
    arms = {}
    for label, traj in trajectories.items():
        ip = '192.168.1.111' if label == 'left' else '192.168.1.214'
        print(f"\nConnecting to {label} arm ({ip})...")
        
        try:
            arm = XArmAPI(ip)
            arm.motion_enable(True)
            arm.set_mode(0)  # Position mode
            arm.set_state(0)  # Ready
            arms[label] = arm
            print(f"✅ Connected to {label} arm")
        except Exception as e:
            print(f"❌ Failed to connect to {label} arm: {e}")
            return 1
    
    # Replay trajectories
    print(f"\n▶️  Starting replay at {speed}x speed...")
    print("   Press robot STOP button to abort")
    print()
    
    try:
        # Get the longest trajectory for timing
        max_samples = max(len(traj['timestamps']) for traj in trajectories.values())
        
        start_time = time.time()
        
        for i in range(max_samples):
            # Calculate target time for this frame
            for label, traj in trajectories.items():
                if i < len(traj['timestamps']):
                    target_time = (traj['timestamps'][i] - traj['timestamps'][0]) / speed
                    break
            
            # Wait until target time
            while time.time() - start_time < target_time:
                time.sleep(0.001)
            
            # Send joint positions to each arm
            for label, traj in trajectories.items():
                if i >= len(traj['timestamps']):
                    continue  # This arm's trajectory is shorter
                
                joint_angles = traj['joint_positions'][i].tolist()
                
                try:
                    arms[label].set_servo_angle(angle=joint_angles, is_radian=False, wait=False)
                except Exception as e:
                    print(f"\n❌ Replay error on {label} arm at frame {i}: {e}")
                    break
            
            # Progress indicator
            if i % 100 == 0:
                progress = (i / max_samples) * 100
                print(f"\rProgress: {progress:.1f}% ({i}/{max_samples} frames)", end='', flush=True)
        
        print("\n\n✅ Replay complete")
    
    except KeyboardInterrupt:
        print("\n\n⚠️ Replay interrupted by user")
    
    finally:
        # Disconnect arms
        for label, arm in arms.items():
            try:
                arm.disconnect()
                print(f"✅ Disconnected from {label} arm")
            except:
                pass
    
    return 0


def validate_trial(trial_id: str, data_dir: str = '/home/joshua/Research/dex-teleop/data'):
    """
    Validate a recorded trial.
    
    Args:
        trial_id: Trial identifier
        data_dir: Directory containing trial files
        
    Returns:
        True if validation passed
    """
    if not HDF5_AVAILABLE:
        print("❌ h5py not installed")
        return False
    
    trial_file = Path(data_dir) / f"trial_{trial_id}.h5"
    if not trial_file.exists():
        print(f"❌ Trial not found: {trial_file}")
        return False
    
    print(f"Validating trial: {trial_id}")
    print("="*60)
    
    try:
        with h5py.File(trial_file, 'r') as f:
            # Check schema version
            schema_version = f['metadata'].attrs.get('schema_version', 'unknown')
            print(f"✅ Schema version: {schema_version}")
            
            if schema_version != "1.0":
                print(f"⚠️  Unknown schema version: {schema_version}")
            
            # Get active arms from metadata (for recordings with single-arm support)
            active_arms_str = f['metadata'].attrs.get('active_arms', 'left,right')
            active_arms = active_arms_str.split(',')
            print(f"✅ Active arms: {', '.join(active_arms)}")
            
            # Validate each device group (only for active arms)
            devices_to_check = []
            for arm in active_arms:
                devices_to_check.extend([f'xarm_{arm}', f'inspire_{arm}'])
            
            for device in devices_to_check:
                if device not in f:
                    print(f"❌ Missing group: /{device}")
                    return False
                
                group = f[device]
                
                if device.startswith('xarm'):
                    # Check xArm datasets
                    required = ['timestamps_mono', 'joint_positions', 'tcp_poses']
                    for dset_name in required:
                        if dset_name not in group:
                            print(f"❌ {device}: Missing dataset '{dset_name}'")
                            return False
                    
                    n_samples = len(group['timestamps_mono'])
                    joint_shape = group['joint_positions'].shape
                    tcp_shape = group['tcp_poses'].shape
                    
                    print(f"✅ {device}: {n_samples} samples")
                    print(f"   - joint_positions: {joint_shape}")
                    print(f"   - tcp_poses: {tcp_shape}")
                    
                    # Validate shapes
                    if joint_shape[0] != n_samples or joint_shape[1] != 7:
                        print(f"❌ {device}: Invalid joint_positions shape")
                        return False
                    if tcp_shape[0] != n_samples or tcp_shape[1] != 6:
                        print(f"❌ {device}: Invalid tcp_poses shape")
                        return False
                    
                    # Check timestamp monotonicity
                    timestamps = group['timestamps_mono'][:]
                    if not np.all(np.diff(timestamps) >= 0):
                        print(f"❌ {device}: Timestamps not monotonic")
                        return False
                
                elif device.startswith('inspire'):
                    # Check Inspire datasets
                    required = ['timestamps_mono', 'angles']
                    for dset_name in required:
                        if dset_name not in group:
                            print(f"❌ {device}: Missing dataset '{dset_name}'")
                            return False
                    
                    n_samples = len(group['timestamps_mono'])
                    angles_shape = group['angles'].shape
                    
                    print(f"✅ {device}: {n_samples} samples")
                    print(f"   - angles: {angles_shape}")
                    
                    if angles_shape[0] != n_samples or angles_shape[1] != 6:
                        print(f"❌ {device}: Invalid angles shape")
                        return False
                    
                    # Check timestamp monotonicity
                    timestamps = group['timestamps_mono'][:]
                    if not np.all(np.diff(timestamps) >= 0):
                        print(f"❌ {device}: Timestamps not monotonic")
                        return False
                    
                    # Check angle ranges [0, 1000]
                    angles = group['angles'][:]
                    if np.any(angles < 0) or np.any(angles > 1000):
                        print(f"⚠️  {device}: Some angles outside [0, 1000] range")
            
            print("\n✅ Validation passed")
            return True
    
    except Exception as e:
        print(f"❌ Validation error: {e}")
        import traceback
        traceback.print_exc()
        return False


