#!/bin/bash
# Remote Simulator SSH Tunnel Script
# Creates SSH tunnel for ROS2 communication with remote simulator

# Load centralized configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$(python3 "$SCRIPT_DIR/load_config.py" --export-shell)"

echo "🚢 Argo Remote Simulator Tunnel"
echo "==============================="
echo "Remote host: $REMOTE_HOST"
echo "Remote user: $REMOTE_USER"
echo "Remote argo dir: $REMOTE_ARGO_DIR"
echo "ROS domain ID: $ROS_DOMAIN_ID"
echo "Local port: $LOCAL_PORT"
echo "Remote port: $REMOTE_PORT"
echo ""

# Check if SSH key exists
if [ ! -f ~/.ssh/id_rsa ]; then
    echo "⚠️  No SSH key found. Generating one..."
    ssh-keygen -t rsa -b 4096 -f ~/.ssh/id_rsa -N ""
    echo "📋 Copy this public key to the remote machine:"
    echo "   ssh-copy-id $REMOTE_USER@$REMOTE_HOST"
    echo "   Or manually add to ~/.ssh/authorized_keys on remote machine"
    echo ""
    cat ~/.ssh/id_rsa.pub
    echo ""
    exit 1
fi

# Test SSH connection
echo "🔍 Testing SSH connection..."
if ! ssh -o ConnectTimeout=10 -o BatchMode=yes $REMOTE_USER@$REMOTE_HOST "echo 'SSH connection successful'" 2>/dev/null; then
    echo "❌ SSH connection failed!"
    echo "   Make sure you can SSH to the remote machine:"
    echo "   ssh $REMOTE_USER@$REMOTE_HOST"
    echo ""
    echo "   If you need to set up SSH keys:"
    echo "   ssh-copy-id $REMOTE_USER@$REMOTE_HOST"
    exit 1
fi

echo "✅ SSH connection successful"
echo ""

# Kill any existing tunnel
echo "🧹 Cleaning up existing tunnels..."
pkill -f "ssh.*$REMOTE_HOST.*$LOCAL_PORT" 2>/dev/null || true

# Create SSH tunnel
echo "🔗 Creating SSH tunnel..."
echo "   Local port $LOCAL_PORT -> Remote port $REMOTE_PORT"
echo "   Press Ctrl+C to stop the tunnel"
echo ""

# Create tunnel in background
ssh -N -L $LOCAL_PORT:localhost:$REMOTE_PORT $REMOTE_USER@$REMOTE_HOST &
TUNNEL_PID=$!

# Wait a moment for tunnel to establish
sleep 2

# Check if tunnel is working
if ! netstat -ln | grep -q ":$LOCAL_PORT "; then
    echo "❌ Tunnel failed to establish"
    kill $TUNNEL_PID 2>/dev/null
    exit 1
fi

echo "✅ SSH tunnel established (PID: $TUNNEL_PID)"
echo "   ROS2 communication: localhost:$LOCAL_PORT"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Stopping SSH tunnel..."
    kill $TUNNEL_PID 2>/dev/null
    echo "✅ Tunnel stopped"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Keep script running
echo "🔄 Tunnel is active. Keep this terminal open."
echo "   In another terminal, run:"
echo "   export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "   python3 scripts/remote_simulator_launch.py"
echo ""

# Wait for tunnel
wait $TUNNEL_PID
