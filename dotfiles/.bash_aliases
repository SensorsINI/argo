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


# Argo Robot Control Aliases
alias al='make -C /home/orangepi/argo start'
alias aq='make -C /home/orangepi/argo stop'
alias ar='make -C /home/orangepi/argo record'
alias ac='make -C /home/orangepi/argo stop-record'
alias as='make -C /home/orangepi/argo status'
alias ars='make -C /home/orangepi/argo restart'
alias argo_help='bash /home/orangepi/argo/scripts/argo_help.sh'

