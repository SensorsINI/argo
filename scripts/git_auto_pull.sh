#!/bin/bash
# Git auto-pull script for Cursor workspace
# This script can be run manually or triggered by system events

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR" || exit 1

# Check if there are uncommitted changes
if ! git diff-index --quiet HEAD --; then
    echo "⚠️  Warning: You have uncommitted changes. Skipping git pull."
    echo "   Commit or stash your changes before pulling."
    exit 0
fi

# Check if we're on a branch with a remote tracking branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
REMOTE_TRACKING=$(git config --get "branch.${CURRENT_BRANCH}.remote")

if [ -z "$REMOTE_TRACKING" ]; then
    echo "ℹ️  Current branch '$CURRENT_BRANCH' has no remote tracking branch."
    exit 0
fi

# Perform git pull
echo "🔄 Pulling latest changes from remote..."
if git pull --quiet; then
    echo "✅ Git pull completed successfully."
else
    echo "❌ Git pull failed. Check for conflicts or network issues."
    exit 1
fi
