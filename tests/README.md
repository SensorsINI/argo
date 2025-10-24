# Argo Testing Directory

This directory contains various tests for the Argo sailboat system.

## Test Categories

### ROS2 Service Communication Test
Tests to verify that different ROS2 services can communicate with each other via Trigger service calls.

### LoRa Communication Tests
Located in `lora/` subdirectory - testing and analysis scripts for LoRa communication validation.

## Files

- `test_service_server.py` - ROS2 service server that responds to Trigger service calls
- `test_service_client.py` - ROS2 service client that calls the Trigger service
- `test-service-server.service` - Systemd service file for the server
- `test-service-client.service` - Systemd service file for the client
- `test_services.sh` - Test script to install, run, and cleanup the services

## Test Results

✅ **SUCCESS**: The test was completed successfully on 2025-10-05.

### Test Execution Summary

1. **Service Server**: Started successfully and began listening for Trigger service calls
   - Service name: `test_trigger_service`
   - Node name: `test_service_server`
   - Status: Active and running

2. **Service Client**: Successfully called the Trigger service and received response
   - Service call successful: ✅ True
   - Response message: "Test service server responded successfully!"
   - Exit code: 0 (success)

3. **Communication**: Verified bidirectional communication between services
   - Server received and processed the request
   - Client received the expected response
   - Both services logged their interaction properly

## How to Run the Test

### Using the Test Script
```bash
# Install services
./test_services.sh install

# Run the test
./test_services.sh test

# View logs
./test_services.sh logs

# Cleanup
./test_services.sh cleanup
```

### Manual Testing
```bash
# Start server in background
bash -c 'source /opt/ros/humble/setup.bash && python3 test_service_server.py' &

# Run client
bash -c 'source /opt/ros/humble/setup.bash && python3 test_service_client.py'
```

## Technical Details

- **ROS2 Version**: Humble
- **Python Version**: 3.9.13 (system Python)
- **Service Type**: std_srvs/Trigger
- **User**: tobi
- **Environment**: Properly sourced ROS2 setup

## Systemd Integration

The services are configured to run as systemd services under the `tobi` user:

- Server runs as a persistent service (`Type=simple`)
- Client runs as a one-shot service (`Type=oneshot`)
- Proper ROS2 environment sourcing in ExecStart
- Journal logging for debugging

This test confirms that ROS2 services can successfully communicate with each other in the Argo environment, which is essential for the distributed control system architecture.
