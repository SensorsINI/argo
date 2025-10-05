
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
    local force_check=false
    
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -f|--force)
                force_check=true
                shift
                ;;
            *)
                echo "Usage: argo_quick_timer [-f|--force]"
                echo "  -f, --force    Force status check regardless of timer"
                return 1
                ;;
        esac
    done
    
    # Check if quick timer should run (5+ minutes since last check)
    local last_check_file="$HOME/.argo_last_check"
    local current_time=$(date +%s)
    local last_check_time=0
    
    if [ -f "$last_check_file" ]; then
        last_check_time=$(cat "$last_check_file" 2>/dev/null || echo "0")
    fi
    
    local time_diff=$((current_time - last_check_time))
    
    # Run check if forced or if it's been more than 5 minutes (300 seconds)
    if [ "$force_check" = true ] || [ $time_diff -ge 300 ]; then
        if [ "$force_check" = true ]; then
            echo -n "Force running argo_quick_timer, please wait..."
        else
            echo -n "Running argo_quick_timer, please wait..."
        fi
        # Show condensed status and update timestamp
        local full_status=$(python3 ~/argo/launch/argo_lifecycle_manager.py status)
        local node_count=$(echo "$full_status" | grep '🤖 ROS NODES:' | sed 's/🤖 ROS NODES: //')
        local system_info=$(echo "$full_status" | grep '📊 SYSTEM:' | sed 's/📊 SYSTEM: //')
        local status_result="🚢 ARGO: ${node_count} | ${system_info}"
        echo -ne "\r\033[K${status_result}\n"
        echo "$current_time" > "$last_check_file"
    fi
}

PATH="$HOME/argo/launch:$HOME/argo/nodes:$HOME/argo/scripts:$PATH"

# Run Argo status check on shell startup (with quick timer)
# This will only show status if it's been more few minutes since last check
argo_quick_timer
