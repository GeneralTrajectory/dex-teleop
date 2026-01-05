using TMPro;
using UnityEngine;

public class HandStreamHUD : MonoBehaviour
{
    public HandRotationsUdpStreamer streamer;
    public TMP_Text text;

    float _t;
    int _frames;

    void Awake()
    {
        if (text == null) text = GetComponent<TMP_Text>();
    }

    void Update()
    {
        _t += Time.deltaTime; _frames++;
        var fps = _t > 0 ? _frames / _t : 0f;

        if (text != null && streamer != null)
        {
            text.text = $"Hand Stream ACTIVE\n" +
                        $"Target: {streamer.receiverIp}:{streamer.receiverPort}\n" +
                        $"FPS~{fps:0}\n" +
                        $"SendLeft={streamer.sendLeft} SendRight={streamer.sendRight}";
        }
    }
}
