#!/usr/bin/env python3
"""
Test script for Quest hand tracking integration.
Simulates Quest UDP packets to test the hand control system without headset.
"""

import socket
import json
import time
import math


def send_test_packet(sock, hand_label, curls, thumb_flex, thumb_opp):
    """Send a simulated Quest hand tracking packet."""
    
    # Simulate 21 joint quaternions (simplified - all identity)
    qs = [0.0, 0.0, 0.0, 1.0] * 21
    
    # Simulate positions (6 joints with 3D positions)
    pos = [0.0] * 18
    
    # Tracking mask (all tracked)
    mask = [1] * 21
    
    # Create message
    msg = {
        'hand': hand_label,
        'q': qs,
        'pos': pos,
        'm': mask,
        'f': 0,  # Frame number
        't': time.time()
    }
    
    # Send to localhost
    sock.sendto(json.dumps(msg).encode('utf-8'), ('127.0.0.1', 9000))
    print(f"Sent [{hand_label}] curls: {curls}, thumb_flex: {thumb_flex}°, thumb_opp: {thumb_opp}°")


def main():
    """Send test packets simulating various hand poses."""
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("="*60)
    print("Quest Hand Tracking Test Sender")
    print("="*60)
    print("\nSending test packets to localhost:9000")
    print("Run 'python quest_inspire_teleop.py' in another terminal\n")
    
    try:
        # Test sequence
        test_poses = [
            # (hand, curls, thumb_flex, thumb_opp, description)
            ('right', {'Little': 0.0, 'Ring': 0.0, 'Middle': 0.0, 'Index': 0.0, 'Thumb': 0.0}, 0.0, 90.0, "Open hand"),
            ('left', {'Little': 0.0, 'Ring': 0.0, 'Middle': 0.0, 'Index': 0.0, 'Thumb': 0.0}, 0.0, 90.0, "Open hand"),
            
            ('right', {'Little': 3.0, 'Ring': 3.0, 'Middle': 3.0, 'Index': 3.0, 'Thumb': 2.0}, 180.0, 10.0, "Closed fist"),
            ('left', {'Little': 3.0, 'Ring': 3.0, 'Middle': 3.0, 'Index': 3.0, 'Thumb': 2.0}, 180.0, 10.0, "Closed fist"),
            
            ('right', {'Little': 2.5, 'Ring': 2.5, 'Middle': 2.5, 'Index': 0.5, 'Thumb': 0.5}, 30.0, 70.0, "Point"),
            ('left', {'Little': 2.5, 'Ring': 2.5, 'Middle': 2.5, 'Index': 0.5, 'Thumb': 0.5}, 30.0, 70.0, "Point"),
            
            ('right', {'Little': 2.5, 'Ring': 2.5, 'Middle': 2.5, 'Index': 2.0, 'Thumb': 2.0}, 120.0, 30.0, "Pinch"),
            ('left', {'Little': 2.5, 'Ring': 2.5, 'Middle': 2.5, 'Index': 2.0, 'Thumb': 2.0}, 120.0, 30.0, "Pinch"),
        ]
        
        print("Starting test sequence (8 poses)...\n")
        
        for i, (hand, curls, thumb_flex, thumb_opp, desc) in enumerate(test_poses, 1):
            print(f"\n--- Test {i}/8: {desc} ({hand}) ---")
            
            # Send packet multiple times for reliability
            for _ in range(10):
                send_test_packet(sock, hand, curls, thumb_flex, thumb_opp)
                time.sleep(0.033)  # ~30Hz
            
            print(f"Waiting 2 seconds...")
            time.sleep(2.0)
        
        print("\n✅ Test sequence complete!")
        print("Hands should now be in last test pose.")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    finally:
        sock.close()
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())

