# Pointing MMDVMHost at AIBridge

MMDVMHost's `[P25 Network]` section normally points at a remote P25 reflector
service (e.g., P25Gateway). To send P25 traffic to AIBridge instead, edit
MMDVMHost.ini and change three values.

## Edit MMDVMHost.ini

Open the file MMDVMHost actually uses (on WPSD/Pi-Star this is usually
`/etc/mmdvmhost` or `/etc/MMDVM.ini` — check `systemctl status mmdvmhost`
for the exact path it's started with).

Find the `[P25 Network]` block and set:

```ini
[P25 Network]
Enable=1
GatewayAddress=127.0.0.1
GatewayPort=42020
LocalAddress=127.0.0.1
LocalPort=42010
Debug=0
```

- **`GatewayAddress` / `GatewayPort`** — where MMDVMHost *sends* RX traffic.
  Must match `listen.host` / `listen.port` in `/etc/aibridge/config.yaml`.
- **`LocalAddress` / `LocalPort`** — where MMDVMHost *listens* for TX
  traffic from us. AIBridge learns this automatically from the first RX
  packet it sees, so the port value only matters for MMDVMHost itself.
  Pick anything not in use on `127.0.0.1`.

## Disable the real gateway

If you were previously running `P25Gateway` or similar, stop and disable
it so it isn't fighting AIBridge for the same port:

```bash
sudo systemctl stop p25gateway
sudo systemctl disable p25gateway
```

## Restart MMDVMHost and AIBridge

```bash
sudo systemctl restart mmdvmhost
sudo systemctl restart aibridge
sudo journalctl -u aibridge -f
```

You should see `MMDVMHost peer = 127.0.0.1:42010` (or whichever `LocalPort`
you set) in the log within a few seconds. Key up on the radio, say
something, release PTT. With `kind: mock` in the config you'll get a 1-sec
sine pip back as proof of life. With real STT/LLM/TTS configured, a spoken
reply.

## Reverting

To go back to your normal P25 reflector, restore the original
`GatewayAddress` / `GatewayPort`, re-enable P25Gateway, and stop AIBridge:

```bash
sudo systemctl stop aibridge
sudo systemctl disable aibridge
sudo systemctl enable --now p25gateway
sudo systemctl restart mmdvmhost
```
