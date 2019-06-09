# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

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
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
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
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
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
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

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
export PATH=$PATH:~/App/sublime_text_3/
alias gst="git status"
alias gl="git log"
alias lls="ll -s"
alias source_bash="source ~/.bashrc"
# alias python3="export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH && python3"
alias git_pull_mine="git pull --rebase -s recursive -X ours"
alias edit_bash="vim ~/.bashrc"
alias smg="/home/ziv/App/smartgit/bin/smartgit.sh"
alias vel_16='roslaunch velodyne_pointcloud VLP16_points.launch'
alias livox='roslaunch display_lidar_points livox_nodis.launch bd_list:="0TFDFCE00504141&0TFDFCE00504071"'
alias livox_viewer='/home/ziv/App/Livox_viewer_For_Linux16.04/livox_viewer.sh'
alias view_vel='~/Downloads/VeloView-3.5.0-Linux-64bit/bin/VeloView'
alias cd_ws='cd /home/ziv/catkin_ws/src/livox_odom'
alias chinese_in='fcitx restart'
alias livox_bag='rosbag record /livox/lidar'
# added by Anaconda3 installer
#export PATH="$PATH:/home/ziv/anaconda3/bin"
# add pycharm path
export PATH="/home/ziv/App/smartgit/bin:/home/ziv/App/pycharm-community-2018.1.4/bin:$PATH"
export LD_LIBRARY_PATH=/usr/local/cuda-8.0.82/lib64:/usr/local/lib:/usr/lib/OpenNI2/Drivers/:/home/ziv/anaconda3/pkgs/vtk-8.0.1-0/lib/:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-8.0.82/bin:/home/ziv/opt/cmake-3.14.0-rc4-Linux-x86_64/bin:$PATH
export DJIROS_APPID='1005202'
export DJIROS_ENCKEY='e7666379497e235364d54c3ecdf1370cbfa009f72e15f6a64451f38e1e5b83ba'
export PCL_ROOT='/home/ziv/opt/pcl-pcl-1.8.0'
#export PYTHONPATH=file:///usr/lib/python3/dist-packages/:/usr/local/lib/python3.5/dist-packages/:/opt/ros/kinetic/lib/python2.7/dist-packages/:/usr/local/lib/python2.7/dist-packages:$PYTHONPATH
#export PYTHONPATH=$PYTHONPATH:/home/ziv/anaconda3/lib/python3.6/site-packages/
#export PYTHONPATH=/home/ziv/anaconda3/lib/python3.6/site-packages/
#echo ${PYTHONPATH}
source /opt/ros/kinetic/setup.bash
source /home/ziv/catkin_ws/devel/setup.bash
