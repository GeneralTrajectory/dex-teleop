"""
HDF5 data writer with atomic commits.
"""

import os
import time
import json
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, List
import numpy as np

try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False
    print("⚠️ h5py not installed. Install with: pip install h5py")


class HDF5Writer:
    """
    HDF5 writer for teleoperation recordings.
    
    Implements atomic writes using temp file + rename pattern.
    Schema stores synchronized xArm and Inspire Hand data.
    """
    
    def __init__(self, output_dir: str, subject: Optional[str], 
                 task: Optional[str], notes: Optional[str], ephemeral: bool = False,
                 active_arms: Optional[list] = None):
        """
        Initialize the HDF5 writer.
        
        Args:
            output_dir: Directory to store recordings
            subject: Subject ID (e.g., 'S1')
            task: Task name (e.g., 'pick')
            notes: Session notes
            ephemeral: If True, will discard instead of saving
            active_arms: List of active arms ['left'], ['right'], or ['left', 'right']
        """
        if not HDF5_AVAILABLE:
            raise RuntimeError("h5py not installed")
        
        self.output_dir = Path(output_dir)
        self.active_arms = active_arms or ['left', 'right']
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.subject = subject or "unknown"
        self.task = task or "untitled"
        self.notes = notes or ""
        self.ephemeral = ephemeral
        
        self.trial_id = None
        self.temp_path = None
        self.final_path = None
        self.h5file = None
        
        # Data buffers (collect in memory, flush periodically)
        self.buffers = {
            'xarm_left': {'timestamps': [], 'joint_pos': [], 'tcp_pose': []},
            'xarm_right': {'timestamps': [], 'joint_pos': [], 'tcp_pose': []},
            'inspire_left': {'timestamps': [], 'angles': []},
            'inspire_right': {'timestamps': [], 'angles': []},
        }
        
        self.start_time_mono = None
        self.config = {}
    
    def start(self, config: Optional[Dict] = None) -> str:
        """
        Create temp HDF5 file and initialize structure.
        
        Args:
            config: Configuration dict (device IDs, rates, safety params)
            
        Returns:
            trial_id: Unique trial identifier
        """
        # Generate trial ID from timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.trial_id = timestamp
        self.config = config or {}
        
        # Create temp file path
        self.temp_path = self.output_dir / f".tmp_{self.trial_id}.h5"
        self.final_path = self.output_dir / f"trial_{self.trial_id}.h5"
        
        # Create HDF5 file
        self.h5file = h5py.File(str(self.temp_path), 'w')
        
        # Store metadata
        meta = self.h5file.create_group('metadata')
        meta.attrs['subject'] = self.subject
        meta.attrs['task'] = self.task
        meta.attrs['notes'] = self.notes
        meta.attrs['timestamp'] = datetime.now().isoformat()
        meta.attrs['schema_version'] = "1.0"
        meta.attrs['ephemeral'] = self.ephemeral
        meta.attrs['active_arms'] = ','.join(self.active_arms)
        meta.attrs['config'] = json.dumps(self.config)
        
        # Create groups only for active devices
        for arm_label in self.active_arms:
            self.h5file.create_group(f'xarm_{arm_label}')
            self.h5file.create_group(f'inspire_{arm_label}')
        
        self.start_time_mono = time.monotonic()
        
        print(f"📝 Recording to temp file: {self.temp_path.name}")
        if self.ephemeral:
            print("⚠️  EPHEMERAL MODE - will discard on stop")
        
        return self.trial_id
    
    def append_xarm(self, label: str, data_batch: List[Dict]):
        """
        Append xArm data batch to buffer.
        
        Args:
            label: 'left' or 'right'
            data_batch: List of dicts with keys: 't_mono', 'joint_pos', 'tcp_pose'
        """
        if not data_batch:
            return
        
        key = f'xarm_{label}'
        for sample in data_batch:
            self.buffers[key]['timestamps'].append(sample['t_mono'])
            self.buffers[key]['joint_pos'].append(sample['joint_pos'])
            self.buffers[key]['tcp_pose'].append(sample['tcp_pose'])
    
    def append_inspire(self, label: str, data_batch: List[Dict]):
        """
        Append Inspire Hand data batch to buffer.
        
        Args:
            label: 'left' or 'right'
            data_batch: List of dicts with keys: 't_mono', 'angles'
        """
        if not data_batch:
            return
        
        key = f'inspire_{label}'
        for sample in data_batch:
            self.buffers[key]['timestamps'].append(sample['t_mono'])
            self.buffers[key]['angles'].append(sample['angles'])
    
    def _flush_to_disk(self):
        """Write buffered data to HDF5 file."""
        if not self.h5file:
            return
        
        for device, data in self.buffers.items():
            group = self.h5file[device]
            
            if device.startswith('xarm'):
                # xArm data: timestamps, joint_positions, tcp_poses
                if data['timestamps']:
                    timestamps = np.array(data['timestamps'], dtype=np.float64)
                    joint_pos = np.array(data['joint_pos'], dtype=np.float32)
                    tcp_pose = np.array(data['tcp_pose'], dtype=np.float32)
                    
                    # Create or extend datasets
                    if 'timestamps_mono' not in group:
                        group.create_dataset('timestamps_mono', data=timestamps, 
                                           maxshape=(None,), dtype=np.float64)
                        group.create_dataset('joint_positions', data=joint_pos,
                                           maxshape=(None, joint_pos.shape[1]), dtype=np.float32)
                        group.create_dataset('tcp_poses', data=tcp_pose,
                                           maxshape=(None, 6), dtype=np.float32)
                    else:
                        # Extend existing datasets
                        for name, arr in [('timestamps_mono', timestamps),
                                         ('joint_positions', joint_pos),
                                         ('tcp_poses', tcp_pose)]:
                            dset = group[name]
                            old_len = dset.shape[0]
                            new_len = old_len + len(arr)
                            dset.resize((new_len,) + dset.shape[1:])
                            dset[old_len:new_len] = arr
            
            elif device.startswith('inspire'):
                # Inspire data: timestamps, angles
                if data['timestamps']:
                    timestamps = np.array(data['timestamps'], dtype=np.float64)
                    angles = np.array(data['angles'], dtype=np.int16)
                    
                    if 'timestamps_mono' not in group:
                        group.create_dataset('timestamps_mono', data=timestamps,
                                           maxshape=(None,), dtype=np.float64)
                        group.create_dataset('angles', data=angles,
                                           maxshape=(None, 6), dtype=np.int16)
                    else:
                        for name, arr in [('timestamps_mono', timestamps), ('angles', angles)]:
                            dset = group[name]
                            old_len = dset.shape[0]
                            new_len = old_len + len(arr)
                            dset.resize((new_len,) + dset.shape[1:])
                            dset[old_len:new_len] = arr
            
            # Clear buffer after flushing
            for key in data:
                data[key].clear()
        
        # Ensure data is written to disk
        self.h5file.flush()
    
    def finalize(self) -> Optional[str]:
        """
        Finalize recording and atomically rename temp → final.
        
        Returns:
            Path to final file, or None if ephemeral
        """
        if not self.h5file:
            return None
        
        # Flush any remaining data
        self._flush_to_disk()
        
        # Update duration in metadata
        if self.start_time_mono:
            duration = time.monotonic() - self.start_time_mono
            self.h5file['metadata'].attrs['duration_s'] = duration
        
        # Close file
        self.h5file.close()
        self.h5file = None
        
        if self.ephemeral:
            # Discard the file
            if self.temp_path and self.temp_path.exists():
                self.temp_path.unlink()
                print(f"🗑️  Discarded ephemeral recording: {self.temp_path.name}")
            return None
        else:
            # Atomic rename: temp → final
            if self.temp_path and self.temp_path.exists():
                self.temp_path.rename(self.final_path)
                print(f"💾 Saved recording: {self.final_path}")
                return str(self.final_path)
            return None
    
    def discard(self):
        """Delete temp file without saving."""
        if self.h5file:
            self.h5file.close()
            self.h5file = None
        
        if self.temp_path and self.temp_path.exists():
            self.temp_path.unlink()
            print(f"🗑️  Discarded recording: {self.temp_path.name}")
    
    def flush(self):
        """Manually flush buffered data to disk."""
        self._flush_to_disk()


