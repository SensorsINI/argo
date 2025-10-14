
source ~/argo/dotfiles/.bash_aliases

# Enable bash completion for Argo Python scripts
if [ -f ~/argo/dotfiles/.bash_completion_argo ]; then
    source ~/argo/dotfiles/.bash_completion_argo
fi

# Run Argo shutdown status check once per terminal session
# This provides critical battery and shutdown information for both SSH and desktop terminals
if [[ -z "$ARGO_MOTD_SHOWN" ]]; then
    export ARGO_MOTD_SHOWN=1
    # Check if the shutdown status script exists
    if [[ -f ~/argo/scripts/15-argo-shutdown-status ]]; then
        bash ~/argo/scripts/15-argo-shutdown-status
    fi
fi

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
        # Show "please wait" message that can be aborted by pressing Enter
        echo -n "🔍 Checking Argo status (press Enter to skip)..."
        
        # Check if Enter is pressed within short timeout
        if read -t 1.0 -r; then
            # User pressed Enter - abort check
            echo -ne "\r\033[K"  # Clear the line
            return 0
        fi
        
        # Clear the "please wait" message before showing status
        echo -ne "\r\033[K"
        
        # Show condensed status using optimized quick_status command with --quiet flag
        python3 ~/argo/launch/argo_lifecycle_manager.py quick_status --quiet
    fi
}

PATH="$HOME/argo/launch:$HOME/argo/nodes:$HOME/argo/scripts:$HOME/argo/power_control:$PATH"

# Run Argo status check on shell startup (with quick timer)
# This will only show status if it's been more few minutes since last check
argo_quick_timer
