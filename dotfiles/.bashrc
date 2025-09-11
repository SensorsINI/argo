# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# Suppress display errors for remote connections
if [ -n "$SSH_CLIENT" ] || [ -n "$SSH_TTY" ]; then
    # Remote session - suppress display errors
    export DISPLAY="" 2>/dev/null || true
    unset DISPLAY 2>/dev/null || true
else
    # Local session - set display if not already set
    export DISPLAY=${DISPLAY:-:0} 2>/dev/null || true
fi

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=10000
HISTFILESIZE=20000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null 2>&1; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
    PROMPT_DIRTRIM=2
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors 2>/dev/null)" || eval "$(dircolors -b 2>/dev/null)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -ahlF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
# Only define alert alias for local sessions (not over SSH)
if [ -z "$SSH_CLIENT" ] && [ -z "$SSH_TTY" ]; then
    alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'
else
    # For remote sessions, create a simple alert that just prints to terminal
    alias alert='echo "Alert: $(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'
fi

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi


PATH="~/bin:$PATH:$HOME/.local/bin"
# Avoid duplicates
HISTCONTROL=ignoredups:erasedups
# When the shell exits, append to the history file instead of overwriting it
shopt -s histappend

# After each command, append to the history file and reread it
PROMPT_COMMAND="${PROMPT_COMMAND:+$PROMPT_COMMAND$'\n'}history -a; history -c; history -r"
PROMPT_COMMAND+='; printf %s "$PWD" > ~/.storepwd'
PROMPT_COMMAND+='; argo_hourly_timer'

# https://unix.stackexchange.com/questions/685116/case-insensitive-completion-in-bash
bind -s 'set completion-ignore-case on'

# https://superuser.com/questions/608484/remember-bash-directory-stack-across-sessions
# Don't remember directory stacks for subshells, just the top level
# shell.
if [[ -z "$BASH_SESSION_ID" ]]; then
    # Get bash-session the X Windows session manager, if possible.
    if [[ -n "$SHELL_SESSION_ID" ]]; then
        export BASH_SESSION_ID=$SHELL_SESSION_ID
    else
        export BASH_SESSION_ID="DEFAULT"
    fi
    .  ~/.bash_dirs
    load_dirs
fi

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('$HOME/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="$HOME/anaconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

export JAVA_HOME="$HOME/OpenJDK21U-jdk_x64_linux_hotspot_21.0.2_13/jdk-21.0.2+13"
export PATH=${JAVA_HOME}/bin:${PATH}
# setxkbmap -option ctrl:nocaps
# Only run setxkbmap in local sessions (not over SSH)
if [ -z "$SSH_CLIENT" ] && [ -z "$SSH_TTY" ]; then
    setxkbmap -option caps:none 2>/dev/null || true
fi
export VISUAL=vim
source /opt/ros/humble/setup.bash
# echo "restoring last folder in shell $(<~/.storepwd)"
cd "$(<~/.storepwd)"

# Source bash aliases if available
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# Argo service status check and warning
argo_status_check() {
    local launch_status=$(systemctl is-active argo-launch.service 2>/dev/null)
    local record_status=$(systemctl is-active argo-record.service 2>/dev/null)
    local launch_exists=$(systemctl list-unit-files | grep -q "argo-launch.service" && echo "yes" || echo "no")
    
    # Define ROS nodes from argo_launch.py (exact script names)
    local ros_nodes=("anem.py" "pwm.py" "gps.py" "imu.py" "control.py" "battery_water.py" "foxglove_bridge")
    local running_nodes=0
    local total_cpu=0
    local total_mem=0
    local missing_nodes=()
    
    # Check individual ROS nodes and collect stats
    for node in "${ros_nodes[@]}"; do
        # Find processes matching the exact script name and check if they're from the argo scripts directory
        local node_pids=$(pgrep -f "/$node" 2>/dev/null)
        local valid_pids=""
        
        # Filter to only include processes from the argo scripts directory
        for pid in $node_pids; do
            local cmd_path=$(ps -p $pid -o cmd --no-headers 2>/dev/null)
            if [[ "$cmd_path" == *"$HOME/argo/scripts/"* ]]; then
                valid_pids="$valid_pids $pid"
            fi
        done
        
        if [ -n "$valid_pids" ]; then
            running_nodes=$((running_nodes + 1))
            
            # Get stats for each valid PID
            for pid in $valid_pids; do
                local node_stats=$(ps -p $pid -o pid,pcpu,pmem,cmd --no-headers 2>/dev/null)
                if [ -n "$node_stats" ]; then
                    # Extract CPU and memory percentages
                    local cpu=$(echo "$node_stats" | awk '{print $2}')
                    local mem=$(echo "$node_stats" | awk '{print $3}')
                    
                    # Add to totals (remove % sign if present)
                    cpu=${cpu%\%}
                    mem=${mem%\%}
                    total_cpu=$(echo "$total_cpu + $cpu" | bc -l 2>/dev/null || echo "$total_cpu")
                    total_mem=$(echo "$total_mem + $mem" | bc -l 2>/dev/null || echo "$total_mem")
                fi
            done
        else
            missing_nodes+=("$node")
        fi
    done
    
    # Check if this is a manual call (not from hourly timer)
    local is_manual_call=${1:-false}
    
    if [ "$is_manual_call" = "true" ] || [ ${#missing_nodes[@]} -gt 0 ]; then
        # Full detailed output for manual calls or when nodes are missing
        echo -e "\033[1;33m🚢 ARGO STATUS CHECK - $(date '+%Y-%m-%d %H:%M:%S')\033[0m"
        
        # Check systemd services
        echo -e "\033[1;32m📋 SYSTEMD SERVICES:\033[0m"
        
        # Check argo-launch service
        if [ "$launch_exists" = "yes" ]; then
            if [ "$launch_status" = "active" ]; then
                local launch_pid=$(systemctl show argo-launch.service --property=MainPID --value 2>/dev/null)
                if [ -n "$launch_pid" ] && [ "$launch_pid" != "0" ]; then
                    local launch_stats=$(ps -p $launch_pid -o pid,pcpu,pmem,cmd --no-headers 2>/dev/null)
                    if [ -n "$launch_stats" ]; then
                        echo -e "  \033[1;32margo-launch:\033[0m $launch_stats"
                    fi
                else
                    echo -e "  \033[1;32margo-launch:\033[0m \033[1;33mACTIVE\033[0m (no main PID)"
                fi
            else
                echo -e "  \033[1;31margo-launch:\033[0m \033[1;31m$launch_status\033[0m"
            fi
        else
            echo -e "  \033[1;31margo-launch:\033[0m \033[1;31mSERVICE NOT FOUND\033[0m"
        fi
        
        # Check argo-record service
        if [ "$record_status" = "active" ]; then
            local record_pid=$(systemctl show argo-record.service --property=MainPID --value 2>/dev/null)
            if [ -n "$record_pid" ] && [ "$record_pid" != "0" ]; then
                local record_stats=$(ps -p $record_pid -o pid,pcpu,pmem,cmd --no-headers 2>/dev/null)
                if [ -n "$record_stats" ]; then
                    echo -e "  \033[1;32margo-record:\033[0m $record_stats"
                fi
            else
                echo -e "  \033[1;32margo-record:\033[0m \033[1;33mACTIVE\033[0m (no main PID)"
            fi
        else
            echo -e "  \033[1;31margo-record:\033[0m \033[1;31m$record_status\033[0m"
        fi
        
        # Check individual ROS nodes with detailed output
        echo -e "\033[1;32m🤖 ROS NODES:\033[0m"
        for node in "${ros_nodes[@]}"; do
            local node_pids=$(pgrep -f "/$node" 2>/dev/null)
            local valid_pids=""
            
            # Filter to only include processes from the argo scripts directory
            for pid in $node_pids; do
                local cmd_path=$(ps -p $pid -o cmd --no-headers 2>/dev/null)
                if [[ "$cmd_path" == *"$HOME/argo/scripts/"* ]]; then
                    valid_pids="$valid_pids $pid"
                fi
            done
            
            if [ -n "$valid_pids" ]; then
                # Get stats for each valid PID
                for pid in $valid_pids; do
                    local node_stats=$(ps -p $pid -o pid,pcpu,pmem,cmd --no-headers 2>/dev/null)
                    if [ -n "$node_stats" ]; then
                        # Extract CPU and memory percentages
                        local cpu=$(echo "$node_stats" | awk '{print $2}')
                        local mem=$(echo "$node_stats" | awk '{print $3}')
                        
                        # Color code based on resource usage
                        local cpu_color="\033[1;32m"  # Green by default
                        local mem_color="\033[1;32m"  # Green by default
                        
                        # Simple numeric comparison for CPU
                        if [ $(echo "$cpu > 50" | bc -l 2>/dev/null || echo "0") -eq 1 ]; then
                            cpu_color="\033[1;31m"  # Red for high CPU
                        elif [ $(echo "$cpu > 20" | bc -l 2>/dev/null || echo "0") -eq 1 ]; then
                            cpu_color="\033[1;33m"  # Yellow for medium CPU
                        fi
                        
                        # Simple numeric comparison for memory
                        if [ $(echo "$mem > 10" | bc -l 2>/dev/null || echo "0") -eq 1 ]; then
                            mem_color="\033[1;31m"  # Red for high memory
                        elif [ $(echo "$mem > 5" | bc -l 2>/dev/null || echo "0") -eq 1 ]; then
                            mem_color="\033[1;33m"  # Yellow for medium memory
                        fi
                        
                        echo -e "  \033[1;32m$node:\033[0m PID:$pid CPU:${cpu_color}${cpu}%\033[0m MEM:${mem_color}${mem}%\033[0m"
                    fi
                done
            else
                echo -e "  \033[1;31m$node:\033[0m \033[1;31mNOT RUNNING\033[0m"
            fi
        done
        echo -e "\033[1;36m📊 SUMMARY:\033[0m"
        echo -e "  Running nodes: \033[1;32m$running_nodes\033[0m/\033[1;33m${#ros_nodes[@]}\033[0m"
        echo -e "  Total CPU usage: \033[1;32m${total_cpu}%\033[0m"
        echo -e "  Total memory usage: \033[1;32m${total_mem}%\033[0m"
        
        # System load and memory info
        local load_avg=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
        local mem_info=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
        echo -e "  System load: \033[1;32m$load_avg\033[0m"
        echo -e "  System memory: \033[1;32m${mem_info}%\033[0m used"
        
        # Storage check (from argo_launch.py logic)
        local free_gb=$(df / | awk 'NR==2 {printf "%.1f", $4/1024/1024}')
        local used_percent=$(df / | awk 'NR==2 {printf "%.1f", $3/($3+$4)*100}')
        echo -e "  Storage: \033[1;32m${free_gb}GB\033[0m free (\033[1;32m${used_percent}%\033[0m used)"
        
        echo "============================================================"
        
        if [ "$launch_status" = "active" ] || [ "$record_status" = "active" ]; then
            echo -e "  \033[1;31mStop: \033[0maq (or: sudo systemctl stop argo-launch.service argo-record.service)"
        fi
    else
        # Condensed single-line output for hourly automatic runs
        local load_avg=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
        local mem_info=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
        local free_gb=$(df / | awk 'NR==2 {printf "%.1f", $4/1024/1024}')
        
        echo -e "\033[1;33m🚢 ARGO:\033[0m \033[1;32m$running_nodes\033[0m/\033[1;33m${#ros_nodes[@]}\033[0m nodes | CPU:\033[1;32m${total_cpu}%\033[0m MEM:\033[1;32m${total_mem}%\033[0m | Load:\033[1;32m$load_avg\033[0m SysMem:\033[1;32m${mem_info}%\033[0m | Storage:\033[1;32m${free_gb}GB\033[0m free"
    fi
}

# Manual status check (always shows full details)
argo_status() {
    argo_status_check true
}

# Hourly timer for automatic status checks
argo_hourly_timer() {
    # Check if we should run the hourly check
    local last_check_file="$HOME/.argo_last_check"
    local current_time=$(date +%s)
    local last_check_time=0
    
    if [ -f "$last_check_file" ]; then
        last_check_time=$(cat "$last_check_file" 2>/dev/null || echo "0")
    fi
    
    # Run check if it's been more than 1 hour (3600 seconds) or if manually called
    local time_diff=$((current_time - last_check_time))
    if [ $time_diff -ge 3600 ] || [ "$1" = "force" ]; then
        argo_status_check
        echo "$current_time" > "$last_check_file"
    fi
}

# Run Argo status check on shell startup (with hourly timer)
# This will only show status if it's been more than an hour since last check
argo_hourly_timer
