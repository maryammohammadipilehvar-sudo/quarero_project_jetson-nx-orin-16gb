[RTSP-Streams](https://eneo-security.com/media/n98_media_assets/files/RTSP-Konfiguration_eneo.pdf)

`rtsp://<IP>:<PORT>/rtsp/streaming?channel=<CH>&subtype=<STREAM>`

z.B. `rtsp://192.168.0.10:554/rtsp/streaming?channel=1&subtype=0`

1. IP
2. PORT = 554 (RTSP Port) / 80 (vordefinierter Einzelbild-Port)
3. CH = Channel Number 1 / 2 / 3
4. STREAM = 0 (Mainstream) / 1 (Substream) / 2 (Mobilestream) 不同码率
---

quarero 01 camera: 
certifikate: 847489397E8DDE1249B65DD4A1EBF8
Portkonfiguration
HTTP 80 
HTTPS 443
RTSP 554

`rtsp://192.168.1.135:554/rtsp/streaming?channel=2&subtype=0`

---
pipeline test: ``gst-launch-1.0 uridecodebin uri="rtsp://192.168.137.157:554/rtsp/streaming?channel=2&subtype=0" ! fakesink``

? simutanuously rgb thermal image retrive