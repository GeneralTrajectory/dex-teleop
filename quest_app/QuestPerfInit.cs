// Assets/QuestPerfInit.cs
using System;
using System.Reflection;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class QuestPerfInit : MonoBehaviour
{
    void Start()
    {
        // Aim high; runtime will clamp to a supported rate
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 120;

        // Try to request a higher XR display refresh rate if the platform exposes it
        TryRequestHighestDisplayRefreshRate();
    }

    static void TryRequestHighestDisplayRefreshRate()
    {
        var displays = new List<XRDisplaySubsystem>();
        SubsystemManager.GetSubsystems(displays);
        if (displays.Count == 0) return;

        foreach (var d in displays)
        {
            var t = d.GetType();

            // Try variant: float[] GetAvailableDisplayRefreshRates()
            // or bool TryGetAvailableDisplayRefreshRates(out float[])
            float[] available = null;

            // Method 1: TryGetAvailableDisplayRefreshRates(out float[])
            var tryGetAvail = t.GetMethod("TryGetAvailableDisplayRefreshRates",
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            if (tryGetAvail != null)
            {
                var args = new object[] { null };
                if (tryGetAvail.Invoke(d, args) is bool ok && ok)
                    available = args[0] as float[];
            }

            // Method 2: GetAvailableDisplayRefreshRates()
            if (available == null)
            {
                var getAvail = t.GetMethod("GetAvailableDisplayRefreshRates",
                    BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (getAvail != null)
                {
                    available = getAvail.Invoke(d, null) as float[];
                }
            }

            // Method 3: only have current rate
            float best = 0f;
            if (available != null && available.Length > 0)
            {
                foreach (var r in available) if (r > best) best = r;
            }
            else
            {
                // Fallback: try to read current rate so we at least touch API
                var tryGetCurrent = t.GetMethod("TryGetDisplayRefreshRate",
                    BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
                if (tryGetCurrent != null)
                {
                    object[] args = new object[] { 0f };
                    if (tryGetCurrent.Invoke(d, args) is bool ok && ok)
                        best = (float)args[0];
                }
            }

            if (best <= 0f) best = 90f; // reasonable default request

            // Try variants to set/request the rate
            var tryRequest = t.GetMethod("TryRequestDisplayRefreshRate",
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
            var trySet = t.GetMethod("TrySetDisplayRefreshRate",
                BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);

            try
            {
                if (tryRequest != null)
                {
                    tryRequest.Invoke(d, new object[] { best });
                }
                else if (trySet != null)
                {
                    trySet.Invoke(d, new object[] { best });
                }
                // else: no API on this runtime; nothing to do
            }
            catch { /* ignore if unsupported */ }
        }
    }
}
