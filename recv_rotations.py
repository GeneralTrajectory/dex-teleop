# recv_rotations_all.py
import socket, json, math

PORT = 9000

# Rotation joint order (no tips); must match the sender
joint_order = [
    "Wrist","Palm",
    "ThumbMetacarpal","ThumbProximal","ThumbDistal",
    "IndexMetacarpal","IndexProximal","IndexIntermediate","IndexDistal",
    "MiddleMetacarpal","MiddleProximal","MiddleIntermediate","MiddleDistal",
    "RingMetacarpal","RingProximal","RingIntermediate","RingDistal",
    "LittleMetacarpal","LittleProximal","LittleIntermediate","LittleDistal",
]
J = {name:i for i,name in enumerate(joint_order)}

# Position order coming from the sender (fixed)
pos_order = ["Palm","ThumbMetacarpal","ThumbProximal","ThumbDistal","ThumbTip","IndexMetacarpal"]
P = {name:i for i,name in enumerate(pos_order)}

# ===== Verbose options =====
VERBOSE = False  # set True to dump selected joints' quaternions each frame
VERBOSE_JOINTS = ["ThumbProximal","ThumbDistal","IndexProximal","IndexIntermediate","IndexDistal"]

# ===== Helpers =====
def quat_to_axis_angle(x, y, z, w):
    w = max(-1.0, min(1.0, w))
    return 2.0 * math.acos(w)

def get_joint_quat(qs, joint_index):
    i = joint_index * 4
    if i + 3 >= len(qs): return None
    return qs[i], qs[i+1], qs[i+2], qs[i+3]

def get_named_quat(qs, name):
    idx = J.get(name)
    return get_joint_quat(qs, idx) if idx is not None else None

def vget(pos, name):
    i = P[name]*3
    if i+2 >= len(pos): return None
    x,y,z = pos[i], pos[i+1], pos[i+2]
    if x==0.0 and y==0.0 and z==0.0:  # our "missing" sentinel from sender
        return None
    return (x,y,z)

def vsub(a,b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def vdot(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def vlen(a):   return math.sqrt(vdot(a,a))
def vang(a,b):
    la, lb = vlen(a), vlen(b)
    if la==0 or lb==0: return None
    c = max(-1.0, min(1.0, vdot(a,b)/(la*lb)))
    return math.degrees(math.acos(c))

# ===== Metrics =====
def finger_curl(qs, proximal_index, has_intermediate=True):
    qP = get_joint_quat(qs, proximal_index)
    qI = get_joint_quat(qs, proximal_index + 1) if has_intermediate else None
    qD = get_joint_quat(qs, proximal_index + (2 if has_intermediate else 1))
    if qP is None or qD is None: return None
    mag = lambda q: quat_to_axis_angle(*q)
    total = mag(qP) + (mag(qI) if qI else 0.0) + mag(qD)
    return total / math.pi  # ~0..(2..3)

# indices (proximal joints) in joint_order
prox = {
    "Thumb": 3,       # ThumbProximal
    "Index": 6,
    "Middle": 10,
    "Ring": 14,
    "Little": 18,
}

def thumb_flexion(pos):
    tm = vget(pos,"ThumbMetacarpal")
    tp = vget(pos,"ThumbProximal")
    td = vget(pos,"ThumbDistal")
    tt = vget(pos,"ThumbTip")
    if tp is None or td is None: return None
    v_tm_tp = vsub(tp, tm if tm else tp)
    v_tp_td = vsub(td, tp)
    v_td_tt = vsub(tt, td) if tt else None
    a1 = vang(v_tm_tp, v_tp_td)
    a2 = vang(v_tp_td, v_td_tt) if v_td_tt else 0.0
    if a1 is None: return None
    return a1 + (a2 if a2 else 0.0)

def thumb_opposition(pos):
    palm = vget(pos,"Palm")
    tt   = vget(pos,"ThumbTip")
    imcp = vget(pos,"IndexMetacarpal")
    if palm is None or tt is None or imcp is None: return None
    v_pt = vsub(tt, palm)
    v_pi = vsub(imcp, palm)
    return vang(v_pt, v_pi)

# ===== Main =====
def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("", PORT))
    print(f"listening on UDP *:{PORT}")

    expected = len(joint_order)          # 21
    expected_with_tips = expected + 5    # 26 (if includeTips = true)

    while True:
        data, addr = s.recvfrom(65535)
        try:
            msg = json.loads(data.decode("utf-8"))
        except Exception as e:
            print("bad json", e); continue

        hand = msg.get("hand","?")
        qs   = msg.get("q", [])
        pos  = msg.get("pos", [])
        mask = msg.get("m")
        n    = len(qs)//4

        if n not in (expected, expected_with_tips):
            print(f"{hand:5s} joints={n} (unexpected length)"); continue

        # curls for fingers (thumb uses 2 joints)
        curls = {
            "Thumb":  round(finger_curl(qs, prox["Thumb"], has_intermediate=False), 2) if finger_curl(qs, prox["Thumb"], has_intermediate=False) is not None else None,
            "Index":  round(finger_curl(qs, prox["Index"]), 2)  if finger_curl(qs, prox["Index"])  is not None else None,
            "Middle": round(finger_curl(qs, prox["Middle"]), 2) if finger_curl(qs, prox["Middle"]) is not None else None,
            "Ring":   round(finger_curl(qs, prox["Ring"]), 2)   if finger_curl(qs, prox["Ring"])   is not None else None,
            "Little": round(finger_curl(qs, prox["Little"]), 2) if finger_curl(qs, prox["Little"]) is not None else None,
        }

        flex = thumb_flexion(pos)
        opp  = thumb_opposition(pos)
        tracked = sum(mask) if isinstance(mask, list) else None

        f = msg.get("f"); t = msg.get("t")
        meta = (f" f={f}" if f is not None else "") + (f" t={t:.6f}" if isinstance(t,(int,float)) else "")
        trk = f" tracked={tracked}/{n}" if tracked is not None else ""

        print(f"{hand:5s} joints={n}{trk} curls={curls} "
              f"thumbFlex={None if flex is None else round(flex,1)}° "
              f"thumbOpp={None if opp is None else round(opp,1)}°{meta}")

        if VERBOSE:
            # dump select joints' local quaternions
            for name in VERBOSE_JOINTS:
                q = get_named_quat(qs, name)
                if q:
                    print(f"   {name:17s} q=({q[0]:+.3f},{q[1]:+.3f},{q[2]:+.3f},{q[3]:+.3f})")

if __name__ == "__main__":
    main()
