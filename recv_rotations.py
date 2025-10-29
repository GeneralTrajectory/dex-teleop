import socket, json, math

PORT = 9000

# Order must match the sender (no tips)
joint_order = [
    "Wrist","Palm",
    "ThumbMetacarpal","ThumbProximal","ThumbDistal",
    "IndexMetacarpal","IndexProximal","IndexIntermediate","IndexDistal",
    "MiddleMetacarpal","MiddleProximal","MiddleIntermediate","MiddleDistal",
    "RingMetacarpal","RingProximal","RingIntermediate","RingDistal",
    "LittleMetacarpal","LittleProximal","LittleIntermediate","LittleDistal",
]

# Indices of *proximal* joints in that order (thumb has no "intermediate")
fingers = {
    "Thumb": 3,   # ThumbProximal
    "Index": 6,
    "Middle": 10,
    "Ring": 14,
    "Little": 18,
}

def quat_to_axis_angle(x, y, z, w):
    w = max(-1.0, min(1.0, w))
    return 2.0 * math.acos(w)

def get_joint_quat(qs, joint_index):
    i = joint_index * 4
    if i + 3 >= len(qs):  # safety
        return None
    return qs[i], qs[i+1], qs[i+2], qs[i+3]

def finger_curl(qs, proximal_index, has_intermediate=True):
    """
    Crude curl metric from local rotations:
      - Thumb: proximal + distal
      - Others: proximal + intermediate + distal
    """
    qP = get_joint_quat(qs, proximal_index)
    qI = get_joint_quat(qs, proximal_index + 1) if has_intermediate else None
    qD = get_joint_quat(qs, proximal_index + (2 if has_intermediate else 1))
    if qP is None or qD is None:
        return None
    mag = lambda q: quat_to_axis_angle(*q)
    total = mag(qP) + (mag(qI) if qI else 0.0) + mag(qD)
    return total / math.pi  # ~0..(2..3)

def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("", PORT))
    print(f"listening on UDP *:{PORT}")

    expected_no_tips = len(joint_order)       # 21
    expected_with_tips = expected_no_tips + 5 # 26

    while True:
        data, addr = s.recvfrom(65535)
        try:
            msg = json.loads(data.decode("utf-8"))
        except Exception as e:
            print("bad json from", addr, "len", len(data), "err:", e)
            continue

        hand = msg.get("hand", "?")
        qs = msg.get("q", [])
        mask = msg.get("m")  # optional 0/1 list same length as joints
        n = len(qs) // 4

        if n not in (expected_no_tips, expected_with_tips):
            print(f"{hand:5s} joints={n} (unexpected length)")
            continue

        # Thumb has no intermediate
        curls = {}
        for name, prox_idx in fingers.items():
            has_intermediate = (name != "Thumb")
            val = finger_curl(qs, prox_idx, has_intermediate=has_intermediate)
            curls[name] = None if val is None else round(val, 2)

        # Count tracked joints if mask present
        tracked = sum(mask) if isinstance(mask, list) else None
        tracked_info = f" tracked={tracked}/{n}" if tracked is not None else ""

        f = msg.get("f"); t = msg.get("t")
        extra = f" f={f}" if f is not None else ""
        extra += f" t={t:.6f}" if isinstance(t, (int, float)) else ""

        print(f"{hand:5s} joints={n}{tracked_info} curls={curls}{extra}")

if __name__ == "__main__":
    main()

