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


# Argo Robot Control Aliases - Improved ROS2 Native Approach
alias al='/home/orangepi/argo/launch/argo_start.sh'           # Start Argo nodes
alias aq='/home/orangepi/argo/launch/argo_stop.sh'            # Stop Argo nodes  
alias ars='/home/orangepi/argo/launch/argo_restart.sh'        # Restart Argo nodes
alias as='/home/orangepi/argo/launch/argo_status.sh'          # Show Argo status
alias am='/home/orangepi/argo/launch/argo_monitor.sh'  # Monitor mode - continuous status display
alias ar='if systemctl is-active --quiet argo-launch.service; then timeout 10 ros2 service call /argo/recording/start std_srvs/srv/Trigger; else echo "❌ Error: argo-launch.service is not running"; echo "   Start it first with: al (or: sudo systemctl start argo-launch.service)"; fi'  # Start recording
alias ac='if systemctl is-active --quiet argo-launch.service; then timeout 10 ros2 service call /argo/recording/stop std_srvs/srv/Trigger; else echo "❌ Error: argo-launch.service is not running"; echo "   Start it first with: al (or: sudo systemctl start argo-launch.service)"; fi'   # Stop recording
alias ah='bash /home/orangepi/argo/launch/argo_help.sh' # show argo help
alias ag='sudo python3 /home/orangepi/argo/launch/argo_gui.py' # start argo gui
alias alog='journalctl -u argo-launch.service -f --no-pager' # show argo launch logs
alias asim='python3 /home/orangepi/argo/launch/argo_lifecycle_manager.py simulate_local' # start argo in LOCAL simulation mode
alias asimr='python3 /home/orangepi/argo/launch/argo_lifecycle_manager.py simulate_remote' # start argo in REMOTE simulation mode
alias cleanbags='find /home/orangepi/argo/bags -type f -mtime +3 -exec rm -f {} \;' # clean old bags