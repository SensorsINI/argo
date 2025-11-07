#!/bin/bash
# Debug script for testing Foxglove WebSocket connection from remote host
# Run this on tobidh87 to debug the connection to argo via router

echo "🔍 Foxglove WebSocket Connection Debug"
echo "======================================"
echo ""

# Connection details
ROUTER_HOST="sensors-tobi-router.lan.ini.uzh.ch"
PORT="8765"
WS_URL="ws://${ROUTER_HOST}:${PORT}"

echo "Connection Details:"
echo "  Router: ${ROUTER_HOST}"
echo "  Port: ${PORT}"
echo "  WebSocket URL: ${WS_URL}"
echo ""

# Test 1: Basic network connectivity
echo "1. Testing network connectivity to router..."
if ping -c 2 -W 2 "${ROUTER_HOST}" > /dev/null 2>&1; then
    echo "   ✅ Router is reachable"
else
    echo "   ❌ Router is NOT reachable"
    echo "   Check network connection and DNS resolution"
fi
echo ""

# Test 2: Port connectivity
echo "2. Testing TCP port connectivity..."
if command -v nc > /dev/null 2>&1; then
    if timeout 3 nc -zv "${ROUTER_HOST}" "${PORT}" 2>&1; then
        echo "   ✅ Port ${PORT} is open and accepting connections"
    else
        echo "   ❌ Port ${PORT} is NOT accessible"
        echo "   Check router port forwarding configuration"
    fi
elif command -v telnet > /dev/null 2>&1; then
    echo "   Testing with telnet (press Ctrl+] then 'quit' to exit)..."
    timeout 3 telnet "${ROUTER_HOST}" "${PORT}" 2>&1 | head -5
else
    echo "   ⚠️  nc (netcat) or telnet not available for port testing"
    echo "   Install with: sudo apt install netcat-openbsd (or telnet)"
fi
echo ""

# Test 3: WebSocket handshake test (if tools available)
echo "3. Testing WebSocket connection..."
if command -v wscat > /dev/null 2>&1; then
    echo "   Using wscat to test WebSocket..."
    timeout 5 wscat -c "${WS_URL}" 2>&1 | head -10 || echo "   Connection failed or timed out"
elif command -v curl > /dev/null 2>&1; then
    echo "   Using curl to test HTTP upgrade (WebSocket handshake)..."
    curl -v --no-buffer \
         -H "Connection: Upgrade" \
         -H "Upgrade: websocket" \
         -H "Sec-WebSocket-Version: 13" \
         -H "Sec-WebSocket-Key: $(openssl rand -base64 16)" \
         "http://${ROUTER_HOST}:${PORT}" 2>&1 | head -20
else
    echo "   ⚠️  wscat or curl not available for WebSocket testing"
    echo "   Install wscat: npm install -g wscat"
    echo "   Or use: curl (usually pre-installed)"
fi
echo ""

# Test 4: DNS resolution
echo "4. Testing DNS resolution..."
if host "${ROUTER_HOST}" > /dev/null 2>&1; then
    echo "   ✅ DNS resolution works:"
    host "${ROUTER_HOST}" | head -3
else
    echo "   ❌ DNS resolution failed"
    echo "   Check if sensors-tobi-router.lan.ini.uzh.ch resolves correctly"
fi
echo ""

# Test 5: Check if port is in firewall/iptables (if on remote host)
echo "5. Network troubleshooting tips:"
echo "   - Router should forward port ${PORT} to 10.0.0.3:8765"
echo "   - Argo's foxglove_bridge should be listening on 0.0.0.0:8765"
echo "   - Check router firewall rules"
echo "   - Verify router external IP matches sensors-tobi-router.lan.ini.uzh.ch"
echo ""

# Test 6: Test from local network vs remote
echo "6. Connection path:"
echo "   Remote Host (tobidh87)"
echo "   → ${ROUTER_HOST}:${PORT}"
echo "   → Router forwards to → 10.0.0.3:8765 (Argo)"
echo "   → foxglove_bridge on Argo"
echo ""

echo "💡 Quick test commands:"
echo "   # Test port connectivity:"
echo "   nc -zv ${ROUTER_HOST} ${PORT}"
echo ""
echo "   # Test WebSocket with wscat (if installed):"
echo "   wscat -c ${WS_URL}"
echo ""
echo "   # Test with curl:"
echo "   curl -v --no-buffer -H 'Connection: Upgrade' -H 'Upgrade: websocket' http://${ROUTER_HOST}:${PORT}"
echo ""
