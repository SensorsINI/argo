Zerotier VPN service for argo

zerotier.com
tobi@ini.uzh.ch
(password in tobi's password mananger)

argo network id: 41d49af6c243480a (active network)

zerotier installed on argo via curl method https://www.zerotier.com/download/

## Current Connection Status
- **ZeroTier Node ID**: 3c69f48232
- **Network Status**: OK (authorized and connected)
- **Assigned IP**: 10.144.192.18/16
- **Interface**: ztw4lntpwi
- **SSH Connection**: Preserved (10.0.0.3 on wlan0)
- **Service Status**: Enabled (auto-starts on boot)
- **Persistence**: ✅ Network membership persists across reboots

## SSH Access via ZeroTier

### Direct SSH Connection
```bash
# SSH directly via ZeroTier IP
ssh orangepi@10.144.192.18

# Or use the SSH config alias
ssh argo-zerotier
```

### SSH Config Entry
The Orange Pi has been configured with an SSH config entry for easy access:
```bash
Host argo-zerotier
     HostName 10.144.192.18
     User orangepi
     ForwardX11 yes
     ForwardX11Trusted yes
     ServerAliveInterval 30
     ServerAliveCountMax 6
     IdentityFile ~/.ssh/id_rsa
```

### Cursor Remote-SSH Usage
You can now use Cursor's Remote-SSH extension with:
- **Host**: `argo-zerotier` (uses SSH config)
- **Or direct IP**: `10.144.192.18`
- **User**: `orangepi`

## Persistence & Boot Behavior

### ✅ Automatic Reconnection
- **ZeroTier service**: Enabled and auto-starts on boot
- **Network membership**: Persists in `/var/lib/zerotier-one/networks.d/`
- **IP assignment**: Same IP `10.144.192.18` after reboot
- **SSH access**: Available immediately after boot via ZeroTier IP

### Boot Sequence
1. **System boots** → ZeroTier service starts automatically
2. **Network connects** → Joins `41d49af6c243480a` network
3. **IP assigned** → Receives `10.144.192.18/16`
4. **SSH ready** → Accessible via `argo-zerotier` or `10.144.192.18`

## Network Management Commands
```bash
# Check network status
sudo zerotier-cli listnetworks

# Check interface details
ip addr show ztw4lntpwi

# Test connectivity to other devices on the network
ping 10.144.235.8  # Your remote computer

# Join the network (if needed)
sudo zerotier-cli join 41d49af6c243480a

# Leave a network (if needed)
sudo zerotier-cli leave <network_id>

# Check service status
sudo systemctl status zerotier-one

# Restart service (if needed)
sudo systemctl restart zerotier-one
```


