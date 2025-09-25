
source ~/argo/dotfiles/.bash_aliases
# Argo service status check and warning (deprecated - use argo_status instead)
argo_status_check() {
    echo "⚠️  argo_status_check() is deprecated. Use 'argo_status' instead."
    argo_status
}

# Manual status check (always shows full details)
argo_status() {
    python3 ~/argo/launch/argo_lifecycle_manager.py status
}

# Quick timer for automatic status checks
argo_quick_timer() {
    # Check if quick timer should run (5+ minutes since last check)
    local last_check_file="$HOME/.argo_last_check"
    local current_time=$(date +%s)
    local last_check_time=0
    
    if [ -f "$last_check_file" ]; then
        last_check_time=$(cat "$last_check_file" 2>/dev/null || echo "0")
    fi
    
    local time_diff=$((current_time - last_check_time))
    
    # Run check if it's been more than 5 minutes (300 seconds)
    if [ $time_diff -ge 300 ]; then
        # Show condensed status and update timestamp
        echo "🚢 ARGO: $(python3 ~/argo/launch/argo_lifecycle_manager.py status | grep '📊 SYSTEM:' | sed 's/📊 SYSTEM: //')"
        echo "$current_time" > "$last_check_file"
    fi
}

PATH="$HOME/argo/launch:$HOME/argo/nodes:$PATH"

# Run Argo status check on shell startup (with quick timer)
# This will only show status if it's been more few minutes since last check
argo_quick_timer
