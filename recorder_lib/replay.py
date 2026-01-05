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

try:
    from scipy.ndimage import gaussian_filter1d
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("⚠️ scipy not installed. Install with: pip install scipy")

# Add parent path for local imports
sys.path.insert(0, str(Path(__file__).parent.parent))


def _get_default_data_dir() -> str:
    """Get default data directory (relative to this module)."""
    return str(Path(__file__).parent.parent / 'data')


def list_trials(data_dir: str = None):
    """
    List all available trials.
    
    Args:
        data_dir: Directory containing trial files (default: ./data)
    """
    if data_dir is None:
        data_dir = _get_default_data_dir()
    
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
                data_dir: str = None,
                smooth_sigma: float = 5.0,
                collision_drop_threshold: float = 0.80):
    """
    Replay a recorded trial.
    
    Args:
        trial_id: Trial identifier
        speed: Playback speed multiplier (1.0 = real-time)
        dry_run: If True, don't move robots (just validate and plot)
        data_dir: Directory containing trial files (default: ./data)
        smooth_sigma: Standard deviation for Gaussian smoothing (0 to disable)
        collision_drop_threshold: If collision detected after this progress (0.0-1.0),
                                  open gripper and return success. Set to 1.0 to disable.
    """
    if data_dir is None:
        data_dir = _get_default_data_dir()
    
    if not HDF5_AVAILABLE:
        print("❌ h5py not installed")
        return 1
    
    # Try resolving the file path in multiple ways
    possible_paths = [
        Path(data_dir) / trial_id,                        # Exact match (e.g. full filename)
        Path(data_dir) / f"{trial_id}.h5",                # ID + extension
        Path(data_dir) / f"trial_{trial_id}.h5",          # Standard prefix format
    ]
    
    trial_file = None
    for p in possible_paths:
        if p.exists():
            trial_file = p
            break
            
    if not trial_file:
        print(f"❌ Trial not found. Searched:")
        for p in possible_paths:
            print(f"   - {p}")
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
            print(f"  Smoothing sigma: {smooth_sigma}")
            print()
            
            # Load xArm trajectories (only for active arms)
            trajectories = {}
            for arm_label in active_arms:
                device = f'xarm_{arm_label}'
                if device in f and 'timestamps_mono' in f[device]:
                    raw_joints = f[device]['joint_positions'][:]
                    
                    # Apply smoothing if requested
                    if SCIPY_AVAILABLE and smooth_sigma > 0:
                        print(f"   Processing {arm_label} arm: Smoothing joints (sigma={smooth_sigma})...")
                        smoothed_joints = gaussian_filter1d(raw_joints, sigma=smooth_sigma, axis=0)
                    else:
                        smoothed_joints = raw_joints

                    trajectories[arm_label] = {
                        'timestamps': f[device]['timestamps_mono'][:],
                        'joint_positions': smoothed_joints,
                        'raw_joint_positions': raw_joints, # Keep raw for debug if needed
                        'tcp_poses': f[device]['tcp_poses'][:],
                    }
                    n_samples = len(trajectories[arm_label]['timestamps'])
                    print(f"✅ Loaded {arm_label} arm: {n_samples} samples")
            
            # Load Inspire trajectories (only for active arms)
            inspire_traj = {}
            for hand_label in active_arms:
                device = f'inspire_{hand_label}'
                if device in f and 'timestamps_mono' in f[device]:
                    inspire_traj[hand_label] = {
                        'timestamps': f[device]['timestamps_mono'][:],
                        'angles': f[device]['angles'][:],
                    }
                    n_samples = len(inspire_traj[hand_label]['timestamps'])
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
            arm.set_mode(1)  # Servo mode for real-time streaming
            arm.set_state(0)  # Ready
            arms[label] = arm
            print(f"✅ Connected to {label} arm")
        except Exception as e:
            print(f"❌ Failed to connect to {label} arm: {e}")
            return 1
    
    # Ensure arms are at known HOME pose before starting replay
    # This prevents a sudden jump from an arbitrary pose to the trajectory start
    HOME_POSE = [275.0, 0.0, 670.0, 180.0, -1.0, 0.0]  # [x,y,z,roll,pitch,yaw]
    home_speed = int(os.environ.get('REPLAY_HOME_SPEED', '150'))
    print("\n🏠 Moving arm(s) to HOME pose before replay...")
    for label, arm in arms.items():
        try:
            # Use position mode for a planned move to HOME
            arm.set_mode(0)
            arm.set_state(0)
            ret = arm.set_position(*HOME_POSE, speed=home_speed, wait=True, timeout=20)
            if ret != 0:
                print(f"⚠️  {label} arm home move returned code {ret}")
            else:
                print(f"✅ {label} arm at HOME pose")
        except Exception as e:
            print(f"⚠️  Failed to move {label} arm to HOME pose: {e}")
        finally:
            # Return to servo mode for streaming
            try:
                arm.set_mode(1)
                arm.set_state(0)
            except Exception:
                pass
    time.sleep(0.2)
    
    # Connect to Inspire hands if trajectories are available
    inspire_devices = {}
    hand_indices = {}
    if 'inspire_traj' in locals() and len(inspire_traj) > 0:
        try:
            inspire_path = os.environ.get('INSPIRE_HANDS_PATH', '')
            if inspire_path and inspire_path not in sys.path:
                sys.path.insert(0, inspire_path)
            from inspire_hand import InspireHand
            replay_hand_speed = int(os.environ.get('REPLAY_INSPIRE_SPEED', '1000'))
            print("\n🤝 Connecting to Inspire Hand(s) for replay...")
            for label, traj in inspire_traj.items():
                port = '/dev/ttyUSB1' if label == 'left' else '/dev/ttyUSB0'
                try:
                    hand = InspireHand(port=port, slave_id=1, debug=False)
                    hand.open()
                    hand.set_all_finger_speeds(replay_hand_speed)
                    inspire_devices[label] = hand
                    hand_indices[label] = 0
                    print(f"✅ Connected to {label} Inspire Hand ({port})")
                except Exception as e:
                    print(f"⚠️  Failed to connect to Inspire {label}: {e}")
        except Exception as e:
            print(f"⚠️  Inspire library not available: {e}")

    # Prime Inspire hands to first recorded angles to avoid sudden jump on first send
    for label, hand in inspire_devices.items():
        try:
            if label in inspire_traj and len(inspire_traj[label]['angles']) > 0:
                first_angles = inspire_traj[label]['angles'][0].astype(int).tolist()
                hand.modbus.write_multiple_registers(1486, first_angles)
                print(f"✅ {label} hand primed to initial angles")
        except Exception as e:
            print(f"⚠️  Failed to prime {label} hand: {e}")

    # Replay trajectories
    print(f"\n▶️  Starting replay at {speed}x speed...")
    print("   Press robot STOP button to abort")
    if collision_drop_threshold < 1.0:
        print(f"   Collision drop enabled: will drop object if collision after {collision_drop_threshold*100:.0f}%")
    print()
    
    # Track collision state
    collision_detected = False
    collision_progress = 0.0
    
    try:
        # Get the longest trajectory for timing
        max_samples = max(len(traj['timestamps']) for traj in trajectories.values())
        
        start_time = time.time()
        
        for i in range(max_samples):
            # Calculate progress for collision handling
            progress = i / max_samples
            
            # Check for collision on all arms (error code 31 = collision)
            for label, arm in arms.items():
                try:
                    if arm.error_code == 31:
                        collision_detected = True
                        collision_progress = progress
                        print(f"\n⚠️  Collision detected on {label} arm at {progress*100:.1f}% progress!")
                        
                        if progress >= collision_drop_threshold:
                            print(f"   ✅ Progress >= {collision_drop_threshold*100:.0f}% threshold - will drop object")
                        else:
                            print(f"   ❌ Progress < {collision_drop_threshold*100:.0f}% threshold - aborting")
                        break
                except Exception:
                    pass  # Error code read failed, continue
            
            if collision_detected:
                break
            
            # Calculate target time for this frame
            for label, traj in trajectories.items():
                if i < len(traj['timestamps']):
                    target_time = (traj['timestamps'][i] - traj['timestamps'][0]) / speed
                    break
            
            # Wait until target time
            while time.time() - start_time < target_time:
                time.sleep(0.001)
            
            # Send joint positions to each arm (servo joint streaming preferred)
            for label, traj in trajectories.items():
                if i >= len(traj['timestamps']):
                    continue  # This arm's trajectory is shorter
                
                joint_angles = traj['joint_positions'][i].tolist()
                
                try:
                    arm = arms[label]
                    ret = None
                    if hasattr(arm, 'set_servo_angle_j'):
                        # Servo joint streaming (real-time)
                        ret = arm.set_servo_angle_j(joint_angles, is_radian=False)
                    else:
                        # Fallback: try regular set_servo_angle
                        ret = arm.set_servo_angle(angle=joint_angles, is_radian=False, wait=False)
                    
                    if ret not in (None, 0):
                        # Attempt to re-enable servo mode once if needed
                        try:
                            arm.set_mode(1)
                            arm.set_state(0)
                        except Exception:
                            pass
                except Exception as e:
                    print(f"\n❌ Replay error on {label} arm at frame {i}: {e}")
                    break
            
            # Send Inspire hand angles according to their own timestamps
            if inspire_devices:
                elapsed = time.time() - start_time
                for label, hand in inspire_devices.items():
                    try:
                        if label not in inspire_traj:
                            continue
                        ts = inspire_traj[label]['timestamps']
                        angles_arr = inspire_traj[label]['angles']
                        if len(ts) == 0:
                            continue
                        idx = hand_indices.get(label, 0)
                        t0 = ts[0]
                        # Drain all samples whose scheduled time has arrived
                        while idx < len(ts):
                            target_t = (ts[idx] - t0) / max(speed, 1e-6)
                            if elapsed + 1e-4 < target_t:
                                break
                            angles = angles_arr[idx].astype(int).tolist()
                            # Send via Modbus write (same as teleop sender)
                            hand.modbus.write_multiple_registers(1486, angles)
                            idx += 1
                        hand_indices[label] = idx
                    except Exception as e:
                        # Non-fatal; continue replay
                        pass

            # Progress indicator
            if i % 100 == 0:
                pct = (i / max_samples) * 100
                print(f"\rProgress: {pct:.1f}% ({i}/{max_samples} frames)", end='', flush=True)
        
        if not collision_detected:
            print("\n\n✅ Replay complete")
    
    except KeyboardInterrupt:
        print("\n\n⚠️ Replay interrupted by user")
        collision_detected = False  # Don't treat interrupt as collision
    
    # Handle collision outcome
    should_drop_object = collision_detected and collision_progress >= collision_drop_threshold
    
    if collision_detected:
        # Clear error and stop arm motion
        for label, arm in arms.items():
            try:
                arm.clean_error()
                arm.set_mode(0)  # Position mode
                arm.set_state(0)
            except Exception:
                pass
        
        if should_drop_object:
            print("\n🤲 Opening gripper to drop object...")
            # Open all Inspire hands to drop the object
            if 'inspire_devices' in locals():
                for label, hand in inspire_devices.items():
                    try:
                        hand.open_all_fingers()
                        print(f"   ✅ {label} hand opened")
                    except Exception as e:
                        print(f"   ⚠️ Failed to open {label} hand: {e}")
            time.sleep(0.5)  # Let object drop
            print("✅ Object dropped - treating as successful placement")
    
    # Disconnect arms
    for label, arm in arms.items():
        try:
            arm.disconnect()
            print(f"✅ Disconnected from {label} arm")
        except:
            pass
    
    # Disconnect Inspire hands
    if 'inspire_devices' in locals():
        for label, hand in inspire_devices.items():
            try:
                # Keep last pose by default. To restore previous behavior and open on exit,
                # set REPLAY_INSPIRE_OPEN_ON_EXIT=1
                try:
                    open_on_exit = os.environ.get('REPLAY_INSPIRE_OPEN_ON_EXIT', '0').strip().lower() in ('1','true','yes','y')
                    if not open_on_exit and not should_drop_object:
                        # Send final recorded angles once to ensure last pose is latched
                        if 'inspire_traj' in locals() and label in inspire_traj and len(inspire_traj[label]['angles']) > 0:
                            final_angles = inspire_traj[label]['angles'][-1].astype(int).tolist()
                            hand.modbus.write_multiple_registers(1486, final_angles)
                            time.sleep(0.1)
                    elif open_on_exit or should_drop_object:
                        hand.open_all_fingers()
                        time.sleep(0.2)
                except Exception:
                    pass
                hand.close()
                print(f"✅ Disconnected from {label} hand")
            except Exception:
                pass
    
    # Return codes:
    # 0 = success (normal or collision-drop)
    # 1 = early collision (needs retry)
    if collision_detected and not should_drop_object:
        return 1  # Early collision - signal failure
    return 0  # Success


def validate_trial(trial_id: str, data_dir: str = None):
    """
    Validate a recorded trial.
    
    Args:
        trial_id: Trial identifier
        data_dir: Directory containing trial files (default: ./data)
        
    Returns:
        True if validation passed
    """
    if data_dir is None:
        data_dir = _get_default_data_dir()
    
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
