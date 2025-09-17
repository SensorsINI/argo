alias j=autojump
alias relogin="source ~/.bashrc"
alias vimlogin="vim ~/.bashrc && relogin"
alias vimaliases="vim ~/.bash_aliases && relogin"
alias d=pushd
alias u=popd
alias lrt="ls -lhrt"
alias filecount="ls -1|wc -l"
alias cdsensorsini='cd ~/Dropbox/GitHub/SensorsINI'
alias df1="df | egrep '^/'"
alias df2='df -x tmpfs -x efivarfs '
alias mount1="mount | grep -E '^/'|grep -v snapd"
alias sd="sudo shutdown now"


# Argo Robot Control Aliases - Improved ROS2 Native Approach
alias al='/home/orangepi/argo/launch/argo_start.sh'           # Start Argo nodes
alias aq='/home/orangepi/argo/launch/argo_stop.sh'            # Stop Argo nodes  
alias ars='/home/orangepi/argo/launch/argo_restart.sh'        # Restart Argo nodes
alias as='/home/orangepi/argo/launch/argo_status.sh'          # Show Argo status
alias am='python3 /home/orangepi/argo/launch/argo_lifecycle_manager.py monitor'  # Monitor mode
alias ar='if systemctl is-active --quiet argo-launch.service; then ros2 service call /argo/recording/start std_srvs/srv/Empty; else echo "❌ Error: argo-launch.service is not running"; echo "   Start it first with: al (or: sudo systemctl start argo-launch.service)"; fi'  # Start recording
alias ac='if systemctl is-active --quiet argo-launch.service; then ros2 service call /argo/recording/stop std_srvs/srv/Empty; else echo "❌ Error: argo-launch.service is not running"; echo "   Start it first with: al (or: sudo systemctl start argo-launch.service)"; fi'   # Stop recording
alias argo_help='bash /home/orangepi/argo/launch/argo_help.sh'
alias ag='sudo python3 /home/orangepi/argo/launch/argo_gui.py'
