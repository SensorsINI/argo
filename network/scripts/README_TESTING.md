# WiFi Reconnection Testing

This directory contains comprehensive testing tools for the Argo WiFi reconnection system.

## Test Scripts

### `test_wifi_reconnection.sh`
**Main test script** that simulates network disconnection scenarios and monitors system behavior.

**Features:**
- **3-minute duration** with comprehensive testing phases
- **Background execution** - continues even if SSH connection is lost
- **Automatic cleanup** - always restores tobi-wlan connection
- **Power control monitoring** - tracks LED status indicators
- **Comprehensive logging** - detailed logs for analysis

**Test Phases:**
1. **Baseline Monitoring (30s)** - Monitor current system state
2. **Disconnection Test** - Simulate tobi-wlan disconnection
3. **Fallback Verification** - Verify uzh-iot takes over
4. **Restoration Test** - Restore tobi-wlan connection
5. **Final Monitoring (30s)** - Verify system stability

### `run_wifi_test.sh`
**Test launcher** that provides easy management of the test process.

**Commands:**
- `start` - Start test in background
- `status` - Check test progress
- `logs` - Watch live test output
- `stop` - Stop running test
- `results` - Show test results

## Usage

### Quick Start
```bash
# Start test in background
make test-wifi-reconnection

# Check progress
make wifi-test-status

# View results
make wifi-test-results
```

### Manual Usage
```bash
# Start test
./network/scripts/run_wifi_test.sh start

# Monitor progress
./network/scripts/run_wifi_test.sh status

# Watch live logs
./network/scripts/run_wifi_test.sh logs

# View results
./network/scripts/run_wifi_test.sh results
```

### Direct Test Execution
```bash
# Run test directly (foreground)
sudo ./network/scripts/test_wifi_reconnection.sh
```

## Expected Behavior

### Network Switching
1. **Initial State**: Connected to tobi-wlan
2. **Disconnection**: tobi-wlan goes down
3. **Fallback**: uzh-iot takes over automatically
4. **Restoration**: tobi-wlan reconnects when available
5. **Final State**: Back on tobi-wlan

### Power Control LED Indicators
- **WiFi Loss**: Alternating red/green LED pattern
- **WiFi Restored**: Return to normal LED pattern
- **Monitoring**: All LED changes logged

### Logging
- **Test Log**: `/var/log.hdd/persistent/wifi-reconnection-test.log`
- **Power Control Log**: `/var/log.hdd/persistent/argo-power-control.log`
- **WiFi Reconnection Log**: `/var/log.hdd/persistent/wifi-reconnect.log`

## Safety Features

### Automatic Cleanup
The test script includes a cleanup function that:
- **Always runs** - even if script is interrupted
- **Restores tobi-wlan** - ensures remote access
- **Multiple retry attempts** - handles connection failures
- **Status verification** - confirms connection is active

### Error Handling
- **Graceful failures** - continues testing even if some steps fail
- **Comprehensive logging** - all errors are recorded
- **Timeout protection** - prevents hanging on network operations
- **Process monitoring** - tracks background processes

## Test Scenarios

### Scenario 1: Normal Operation
- Start with tobi-wlan connected
- Simulate disconnection
- Verify uzh-iot fallback
- Restore tobi-wlan
- Verify system stability

### Scenario 2: SSH Disconnection
- Test runs in background
- SSH connection can be lost
- Test continues independently
- tobi-wlan automatically restored
- Remote access maintained

### Scenario 3: Network Unavailability
- Test handles missing networks gracefully
- Logs all network states
- Continues monitoring
- Restores preferred connection when available

## Monitoring and Analysis

### Real-time Monitoring
```bash
# Watch test progress
make wifi-test-status

# Monitor live logs
./network/scripts/run_wifi_test.sh logs

# Check power control status
systemctl status argo_power_control.service
```

### Post-test Analysis
```bash
# View complete test results
make wifi-test-results

# Check WiFi reconnection logs
tail -f /var/log.hdd/persistent/wifi-reconnect.log

# Analyze power control events
grep -E "(WiFi|LED)" /var/log.hdd/persistent/argo-power-control.log
```

## Troubleshooting

### Test Won't Start
- Check if already running: `make wifi-test-status`
- Verify permissions: script needs sudo access
- Check dependencies: NetworkManager must be running

### Test Hangs
- Check network connectivity: `nmcli connection show --active`
- Verify power control service: `systemctl status argo_power_control.service`
- Review logs for error messages

### Connection Not Restored
- Test includes automatic retry logic
- Check manual restoration: `nmcli connection up tobi-wlan`
- Verify network availability: `nmcli device wifi list`

## Integration

### Makefile Integration
- `test-wifi-reconnection` - Start test
- `wifi-test-status` - Check status
- `wifi-test-results` - View results

### System Integration
- Uses existing WiFi reconnection system
- Monitors power control service
- Integrates with NetworkManager
- Logs to persistent storage

## Development

### Extending Tests
- Add new test phases in `test_wifi_reconnection.sh`
- Modify monitoring duration in configuration section
- Add new network scenarios as needed

### Customization
- Adjust test duration: Change `TEST_DURATION` variable
- Modify log locations: Update `LOG_FILE` variables
- Add new monitoring: Extend `monitor_power_control()` function

## Best Practices

### Before Testing
1. Ensure tobi-wlan is available and working
2. Verify uzh-iot is accessible as fallback
3. Check power control service is running
4. Confirm sufficient disk space for logs

### During Testing
1. Don't interrupt the test unnecessarily
2. Monitor via status commands rather than direct SSH
3. Let the test complete its full cycle
4. Trust the automatic cleanup process

### After Testing
1. Review test results thoroughly
2. Check all log files for issues
3. Verify system returned to normal state
4. Document any unexpected behavior
