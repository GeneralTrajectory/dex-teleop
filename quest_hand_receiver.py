#!/usr/bin/env python3
"""
UDP receiver for Meta Quest hand tracking data.
Receives JSON packets with finger joint rotations and positions.
"""

import socket
import json
import math
from typing import Optional, Dict, Tuple


class QuestHandReceiver:
    """Receives and parses hand tracking data from Meta Quest via UDP."""
    
    def __init__(self, port: int = 9000):
        """
        Initialize UDP receiver.
        
        Args:
            port: UDP port to listen on (default: 9000)
        """
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", port))
        # Non-blocking socket for minimal latency; we'll poll in a loop
        self.socket.setblocking(False)
        
        # Joint order from Quest (must match Unity sender)
        self.joint_order = [
            "Wrist", "Palm",
            "ThumbMetacarpal", "ThumbProximal", "ThumbDistal",
            "IndexMetacarpal", "IndexProximal", "IndexIntermediate", "IndexDistal",
            "MiddleMetacarpal", "MiddleProximal", "MiddleIntermediate", "MiddleDistal",
            "RingMetacarpal", "RingProximal", "RingIntermediate", "RingDistal",
            "LittleMetacarpal", "LittleProximal", "LittleIntermediate", "LittleDistal",
        ]
        self.joint_index = {name: i for i, name in enumerate(self.joint_order)}
        
        # Position order from Quest
        self.pos_order = ["Palm", "ThumbMetacarpal", "ThumbProximal", "ThumbDistal", "ThumbTip", "IndexMetacarpal"]
        self.pos_index = {name: i for i, name in enumerate(self.pos_order)}
        
        # Proximal joint indices for curl calculation
        self.prox = {
            "Thumb": 3,   # ThumbProximal
            "Index": 6,
            "Middle": 10,
            "Ring": 14,
            "Little": 18,
        }
        
        print(f"📡 Quest hand receiver listening on UDP port {port}")
    
    def receive(self) -> Optional[Dict]:
        """
        Receive one UDP packet and parse hand data.
        
        Returns:
            Dict with hand tracking data, or None if no data available
            Format: {
                'hand': 'left' or 'right',
                'curls': {'Thumb': float, 'Index': float, ...},
                'thumb_flexion': float (degrees),
                'thumb_opposition': float (degrees),
                'tracked': int (number of tracked joints)
            }
        """
        try:
            data, addr = self.socket.recvfrom(65535)
            msg = json.loads(data.decode("utf-8"))
            
            hand = msg.get("hand", "?")
            qs = msg.get("q", [])
            pos = msg.get("pos", [])
            mask = msg.get("m", [])
            
            # Validate data
            n_joints = len(qs) // 4
            if n_joints not in (21, 26):  # With or without tips
                return None
            
            # Compute finger curls
            curls = {
                'Thumb': self._finger_curl(qs, self.prox["Thumb"], has_intermediate=False),
                'Index': self._finger_curl(qs, self.prox["Index"]),
                'Middle': self._finger_curl(qs, self.prox["Middle"]),
                'Ring': self._finger_curl(qs, self.prox["Ring"]),
                'Little': self._finger_curl(qs, self.prox["Little"]),
            }
            
            # Compute thumb metrics
            thumb_flex = self._thumb_flexion(pos)
            thumb_opp = self._thumb_opposition(pos)
            
            # Count tracked joints
            tracked = sum(mask) if isinstance(mask, list) else n_joints
            
            return {
                'hand': hand,
                'curls': curls,
                'thumb_flexion': thumb_flex,
                'thumb_opposition': thumb_opp,
                'tracked': tracked,
                'n_joints': n_joints
            }
            
        except socket.timeout:
            return None
        except Exception:
            return None
    
    def _quat_to_axis_angle(self, x, y, z, w):
        """Convert quaternion to axis-angle magnitude."""
        w = max(-1.0, min(1.0, w))
        return 2.0 * math.acos(w)
    
    def _get_joint_quat(self, qs, joint_index):
        """Extract quaternion for a specific joint."""
        i = joint_index * 4
        if i + 3 >= len(qs):
            return None
        return qs[i], qs[i+1], qs[i+2], qs[i+3]
    
    def _finger_curl(self, qs, proximal_index, has_intermediate=True):
        """
        Compute finger curl metric from joint rotations.
        
        Returns:
            Curl metric ~0.0 (open) to ~3.0 (fully curled), or None if data missing
        """
        qP = self._get_joint_quat(qs, proximal_index)
        qI = self._get_joint_quat(qs, proximal_index + 1) if has_intermediate else None
        qD = self._get_joint_quat(qs, proximal_index + (2 if has_intermediate else 1))
        
        if qP is None or qD is None:
            return None
        
        mag = lambda q: self._quat_to_axis_angle(*q)
        total = mag(qP) + (mag(qI) if qI else 0.0) + mag(qD)
        return total / math.pi  # Normalize to ~0..3 range
    
    def _vget(self, pos, name):
        """Get position vector for a named joint."""
        i = self.pos_index.get(name)
        if i is None:
            return None
        i = i * 3
        if i + 2 >= len(pos):
            return None
        x, y, z = pos[i], pos[i+1], pos[i+2]
        if x == 0.0 and y == 0.0 and z == 0.0:  # Missing sentinel
            return None
        return (x, y, z)
    
    def _vsub(self, a, b):
        """Vector subtraction."""
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
    
    def _vdot(self, a, b):
        """Vector dot product."""
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    
    def _vlen(self, a):
        """Vector length."""
        return math.sqrt(self._vdot(a, a))
    
    def _vang(self, a, b):
        """Angle between two vectors in degrees."""
        la, lb = self._vlen(a), self._vlen(b)
        if la == 0 or lb == 0:
            return None
        c = max(-1.0, min(1.0, self._vdot(a, b) / (la * lb)))
        return math.degrees(math.acos(c))
    
    def _thumb_flexion(self, pos):
        """Compute thumb flexion angle from positions."""
        tm = self._vget(pos, "ThumbMetacarpal")
        tp = self._vget(pos, "ThumbProximal")
        td = self._vget(pos, "ThumbDistal")
        tt = self._vget(pos, "ThumbTip")
        
        if tp is None or td is None:
            return None
        
        v_tm_tp = self._vsub(tp, tm if tm else tp)
        v_tp_td = self._vsub(td, tp)
        v_td_tt = self._vsub(tt, td) if tt else None
        
        a1 = self._vang(v_tm_tp, v_tp_td)
        a2 = self._vang(v_tp_td, v_td_tt) if v_td_tt else 0.0
        
        if a1 is None:
            return None
        return a1 + (a2 if a2 else 0.0)
    
    def _thumb_opposition(self, pos):
        """Compute thumb opposition angle from positions."""
        palm = self._vget(pos, "Palm")
        tt = self._vget(pos, "ThumbTip")
        imcp = self._vget(pos, "IndexMetacarpal")
        
        if palm is None or tt is None or imcp is None:
            return None
        
        v_pt = self._vsub(tt, palm)
        v_pi = self._vsub(imcp, palm)
        return self._vang(v_pt, v_pi)
    
    def close(self):
        """Close the UDP socket."""
        self.socket.close()

