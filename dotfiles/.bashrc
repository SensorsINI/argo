
source ~/argo/dotfiles/.bash_aliases
# Argo service status check and warning
argo_status_check() {
    # Call the Python implementation
    local is_manual_call=${1:-false}
    if [ "$is_manual_call" = "true" ]; then
        python3 ~/argo/launch/argo_status_check.py --manual
    else
        python3 ~/argo/launch/argo_status_check.py
    fi
}

# Manual status check (always shows full details)
argo_status() {
    argo_status_check true
}

# Hourly timer for automatic status checks
argo_hourly_timer() {
    # Call the Python implementation for hourly timer
    if [ "$1" = "force" ]; then
        python3 ~/argo/launch/argo_status_check.py --hourly
    else
        python3 ~/argo/launch/argo_status_check.py --hourly
    fi
}

# Run Argo status check on shell startup (with hourly timer)
# This will only show status if it's been more than an hour since last check
argo_hourly_timer
