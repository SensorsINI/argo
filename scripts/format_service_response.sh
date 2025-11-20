#!/bin/bash
# Format ROS2 service response JSON to human-readable format
# Extracts the JSON from std_srvs/srv/Trigger response and formats it with jq

# Read from stdin (piped from ros2 service call)
input=$(cat)

# Extract JSON from message='...' field
# The format is: message='{...json with \n escapes...}'
json=$(echo "$input" | sed -n "s/.*message='\(.*\)')/\1/p")

# If that didn't work, try alternative pattern
if [ -z "$json" ] || [ "$json" = "" ]; then
    json=$(echo "$input" | grep -oP "message='\K[^']*(?=')" | head -1)
fi

# If still no JSON, output original and exit
if [ -z "$json" ] || [ "$json" = "" ]; then
    echo "$input"
    exit 0
fi

# Unescape the JSON (convert \n to actual newlines, handle \\, etc.)
# Use printf to interpret escape sequences
unescaped_json=$(printf '%b\n' "$json")

# Format with jq if available, otherwise just output unescaped JSON
if command -v jq >/dev/null 2>&1; then
    echo "$unescaped_json" | jq '.' 2>/dev/null || echo "$unescaped_json"
else
    # Fallback to python json.tool if jq not available
    echo "$unescaped_json" | python3 -m json.tool 2>/dev/null || echo "$unescaped_json"
fi
