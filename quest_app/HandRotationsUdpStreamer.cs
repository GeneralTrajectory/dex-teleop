using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;

public class HandRotationsUdpStreamer : MonoBehaviour
{
    [Header("Receiver (where to send)")]
    public string receiverIp = "127.0.0.1";   // set to your Mac/Linux IP
    public int receiverPort = 9000;

    [Header("What to send")]
    public bool sendLeft = true;
    public bool sendRight = true;
    public bool includeTips = false;          // rotations: adds 5 fingertip joints (q array becomes 26*4)
    public bool heartbeat = true;             // small keepalive once per second

    UdpClient _udp;
    IPEndPoint _ep;
    XRHandSubsystem _hands;

    // timing
    ulong _frame;
    static readonly double TicksPerSecond = System.Diagnostics.Stopwatch.Frequency;
    double _lastHeartbeatT;
    readonly StringBuilder _sb = new(8192);

    // caches
    readonly Dictionary<XRHandJointID, Pose> _world = new(64);
    XRHandJointID[] _order;

    // rotation order (parents before children)
    static readonly XRHandJointID[] BaseOrder = {
        XRHandJointID.Wrist, XRHandJointID.Palm,
        XRHandJointID.ThumbMetacarpal, XRHandJointID.ThumbProximal, XRHandJointID.ThumbDistal,
        XRHandJointID.IndexMetacarpal, XRHandJointID.IndexProximal, XRHandJointID.IndexIntermediate, XRHandJointID.IndexDistal,
        XRHandJointID.MiddleMetacarpal, XRHandJointID.MiddleProximal, XRHandJointID.MiddleIntermediate, XRHandJointID.MiddleDistal,
        XRHandJointID.RingMetacarpal, XRHandJointID.RingProximal, XRHandJointID.RingIntermediate, XRHandJointID.RingDistal,
        XRHandJointID.LittleMetacarpal, XRHandJointID.LittleProximal, XRHandJointID.LittleIntermediate, XRHandJointID.LittleDistal
    };
    static readonly XRHandJointID[] Tips = {
        XRHandJointID.ThumbTip, XRHandJointID.IndexTip, XRHandJointID.MiddleTip, XRHandJointID.RingTip, XRHandJointID.LittleTip
    };

    // minimal positions for thumb metrics (fixed order):
    // ["Palm","ThumbMetacarpal","ThumbProximal","ThumbDistal","ThumbTip","IndexMetacarpal"]
    static readonly XRHandJointID[] PosOrder = {
        XRHandJointID.Palm,
        XRHandJointID.ThumbMetacarpal,
        XRHandJointID.ThumbProximal,
        XRHandJointID.ThumbDistal,
        XRHandJointID.ThumbTip,            // if not tracked we'll fill identity/zero
        XRHandJointID.IndexMetacarpal
    };

    static readonly Dictionary<XRHandJointID, XRHandJointID> Parent = new() {
        { XRHandJointID.Wrist, XRHandJointID.Wrist }, { XRHandJointID.Palm, XRHandJointID.Wrist },
        { XRHandJointID.ThumbMetacarpal, XRHandJointID.Palm }, { XRHandJointID.ThumbProximal, XRHandJointID.ThumbMetacarpal },
        { XRHandJointID.ThumbDistal, XRHandJointID.ThumbProximal }, { XRHandJointID.ThumbTip, XRHandJointID.ThumbDistal },
        { XRHandJointID.IndexMetacarpal, XRHandJointID.Palm }, { XRHandJointID.IndexProximal, XRHandJointID.IndexMetacarpal },
        { XRHandJointID.IndexIntermediate, XRHandJointID.IndexProximal }, { XRHandJointID.IndexDistal, XRHandJointID.IndexIntermediate },
        { XRHandJointID.IndexTip, XRHandJointID.IndexDistal },
        { XRHandJointID.MiddleMetacarpal, XRHandJointID.Palm }, { XRHandJointID.MiddleProximal, XRHandJointID.MiddleMetacarpal },
        { XRHandJointID.MiddleIntermediate, XRHandJointID.MiddleProximal }, { XRHandJointID.MiddleDistal, XRHandJointID.MiddleIntermediate },
        { XRHandJointID.MiddleTip, XRHandJointID.MiddleDistal },
        { XRHandJointID.RingMetacarpal, XRHandJointID.Palm }, { XRHandJointID.RingProximal, XRHandJointID.RingMetacarpal },
        { XRHandJointID.RingIntermediate, XRHandJointID.RingProximal }, { XRHandJointID.RingDistal, XRHandJointID.RingIntermediate },
        { XRHandJointID.RingTip, XRHandJointID.RingDistal },
        { XRHandJointID.LittleMetacarpal, XRHandJointID.Palm }, { XRHandJointID.LittleProximal, XRHandJointID.LittleMetacarpal },
        { XRHandJointID.LittleIntermediate, XRHandJointID.LittleProximal }, { XRHandJointID.LittleDistal, XRHandJointID.LittleIntermediate },
        { XRHandJointID.LittleTip, XRHandJointID.LittleDistal },
    };

    void Awake()
    {
        _udp = new UdpClient();
        _order = includeTips ? Join(BaseOrder, Tips) : BaseOrder;
    }

    void OnEnable()
    {
        var loader = XRGeneralSettings.Instance?.Manager?.activeLoader;
        _hands = loader?.GetLoadedSubsystem<XRHandSubsystem>();
        if (_hands == null)
            UnityEngine.Debug.LogError("XRHandSubsystem not found. Enable OpenXR (Android) + Meta Quest Support + Hand Tracking + Meta Hand Tracking Aim.");

        Application.onBeforeRender += OnBeforeRenderSend; // low-latency sample
    }

    void OnDisable()
    {
        Application.onBeforeRender -= OnBeforeRenderSend;
        _udp?.Close();
    }

    void Update()
    {
        if (_ep == null && IPAddress.TryParse(receiverIp, out var ip))
            _ep = new IPEndPoint(ip, receiverPort);

        if (heartbeat && _ep != null)
        {
            var now = System.Diagnostics.Stopwatch.GetTimestamp() / TicksPerSecond;
            if (now - _lastHeartbeatT > 1.0)
            {
                _lastHeartbeatT = now;
                var bytes = Encoding.UTF8.GetBytes("{\"hello\":true}");
                _udp.Send(bytes, bytes.Length, _ep);
            }
        }
    }

    void OnBeforeRenderSend()
    {
        if (_hands == null || _ep == null) return;

        _frame++;
        if (sendLeft)  StreamOne(_hands.leftHand,  "left");
        if (sendRight) StreamOne(_hands.rightHand, "right");
    }

    void StreamOne(XRHand hand, string label)
    {
        if (!hand.isTracked) return;

        _world.Clear();
        foreach (var id in _order)
        {
            var j = hand.GetJoint(id);
            if (j.TryGetPose(out var p)) _world[id] = p;
        }

        // Also fetch positions we need for thumb metrics even if not in rotation order (e.g., ThumbTip)
        foreach (var id in PosOrder)
        {
            if (_world.ContainsKey(id)) continue;
            var j = hand.GetJoint(id);
            if (j.TryGetPose(out var p)) _world[id] = p;
        }

        // timestamp (seconds, high precision)
        double tSec = System.Diagnostics.Stopwatch.GetTimestamp() / TicksPerSecond;

        _sb.Length = 0;
        _sb.Append("{\"hand\":\"").Append(label)
           .Append("\",\"f\":").Append(_frame)
           .Append(",\"t\":").Append(tSec.ToString("G17"))
           .Append(",\"m\":["); // mask for rotations

        // rotation mask (1 tracked / 0 filled)
        for (int i = 0; i < _order.Length; i++)
        {
            bool has = _world.ContainsKey(_order[i]);
            if (i > 0) _sb.Append(',');
            _sb.Append(has ? '1' : '0');
        }
        _sb.Append("],\"q\":["); // rotations as local quats, fixed-order

        // rotations (fill identity for missing)
        for (int i = 0; i < _order.Length; i++)
        {
            Pose pose;
            bool has = _world.TryGetValue(_order[i], out pose);
            Quaternion local;
            if (has)
            {
                var id = _order[i];
                var parentId = Parent[id];
                if (parentId != id && _world.TryGetValue(parentId, out var parentPose))
                    local = Quaternion.Inverse(parentPose.rotation) * pose.rotation;
                else
                    local = pose.rotation;
            }
            else local = new Quaternion(0,0,0,1);

            if (i > 0) _sb.Append(',');
            _sb.Append(local.x.ToString("G7")).Append(',')
              .Append(local.y.ToString("G7")).Append(',')
              .Append(local.z.ToString("G7")).Append(',')
              .Append(local.w.ToString("G7"));
        }

        // minimal positions for thumb metrics (fixed order PosOrder)
        _sb.Append("],\"pos\":[");
        for (int i = 0; i < PosOrder.Length; i++)
        {
            var id = PosOrder[i];
            Vector3 v = Vector3.zero;
            if (_world.TryGetValue(id, out var p)) v = p.position;
            if (i > 0) _sb.Append(',');
            _sb.Append(v.x.ToString("G7")).Append(',')
              .Append(v.y.ToString("G7")).Append(',')
              .Append(v.z.ToString("G7"));
        }
        _sb.Append("]}");

        var bytes = Encoding.UTF8.GetBytes(_sb.ToString());
        _udp.Send(bytes, bytes.Length, _ep);
    }

    static XRHandJointID[] Join(XRHandJointID[] a, XRHandJointID[] b)
    {
        var list = new List<XRHandJointID>(a.Length + b.Length);
        list.AddRange(a); list.AddRange(b);
        return list.ToArray();
    }
}
