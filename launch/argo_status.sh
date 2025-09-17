#!/bin/bash
# Argo Status Script - Simple wrapper for checking Argo status

cd "$(dirname "$0")/.."
python3 launch/argo_lifecycle_manager.py status

