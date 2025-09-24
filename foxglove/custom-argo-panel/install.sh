#!/bin/bash

# Argo Sailboat Custom Foxglove Panel Installation Script
# This script sets up the custom panel extension for Foxglove

set -e

echo "🚢 Installing Argo Sailboat Custom Foxglove Panel..."

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "❌ Node.js is not installed. Please install Node.js first."
    echo "   Visit: https://nodejs.org/"
    exit 1
fi

# Check if npm is installed
if ! command -v npm &> /dev/null; then
    echo "❌ npm is not installed. Please install npm first."
    exit 1
fi

echo "✅ Node.js version: $(node --version)"
echo "✅ npm version: $(npm --version)"

# Navigate to the panel directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "📁 Working directory: $(pwd)"

# Install dependencies
echo "📦 Installing dependencies..."
npm install

# Build the extension
echo "🔨 Building the extension..."
npm run build

# Package the extension
echo "📦 Packaging the extension..."
npm run package

echo ""
echo "✅ Installation complete!"
echo ""
echo "🎯 Next steps:"
echo "1. Start your Argo ROS2 system with rosbridge:"
echo "   ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo ""
echo "2. Open Foxglove Studio in your browser:"
echo "   https://studio.foxglove.dev/"
echo ""
echo "3. Connect to your robot:"
echo "   - Click 'Open connection'"
echo "   - Select 'Rosbridge (WebSocket)'"
echo "   - Enter: ws://YOUR_ROBOT_IP:9090"
echo ""
echo "4. Add the custom panel:"
echo "   - Click the '+' button to add a panel"
echo "   - Select 'argo-sailboat-panel' from the list"
echo ""
echo "🔧 For development:"
echo "   npm run dev    # Watch mode for development"
echo "   npm run build  # Build for production"
echo ""
echo "📚 See README.md for detailed usage instructions."

