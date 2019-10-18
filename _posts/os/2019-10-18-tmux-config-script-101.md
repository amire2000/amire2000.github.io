---
layout: post
title: tmux beginner 
categories: os
tags: [tmux]
description: tmux config, usage and scripting example
public: true
image: tmux.png
---

# config
## Change prefix
- changed ctrl-b to something more accessible
```
unbind C-b
set -g prefix C-a
```
&nbsp;  
&nbsp;  
## Bind key to default splits
- Create file under `~/.tmux/dev` for example
- Bind key to run the file

### dev file

```bash
selectp -t 0    # select the first (0) pane
splitw -h -p 50 # split it into two halves

selectp -t 1    # select the new, second (1) pane
splitw -v -p 50 # split it into two halves
selectp -t 0    # go back to the first pane
```

### bind
```
bind D source-file ~/.tmux/dev
```
&nbsp;  
&nbsp;  
## Reload config from run time

```bash
bind r source-file ~/.tmux.conf
```
&nbsp;  
&nbsp;  
## Pane switching with Alt+arrow

```bash
bind -n M-Left select-pane -L
bind -n M-Right select-pane -R
bind -n M-Up select-pane -U
bind -n M-Down select-pane -D
```
&nbsp;  
&nbsp;  
## Mouse mode
- Enabling mouse mode allows you to select windows and different panes by simply clicking on them
  
```
set -g mouse on
```

# Scripting
## Very basic
- open tmux with two pans
  - each pan execute command
- using `send-keys` to execute commands

```bash
tmux  new-session -d
# Add top pane status , place to print/show pane title
tmux set -g pane-border-status top
tmux split-window -v
tmux select-pane -t 0
# Print pane title
tmux send-keys "printf '\033]2;main\033\\'" ENTER 
tmux send-keys "clear" ENTER
tmux send-keys "cd ~/tmp; ls -l" ENTER

tmux select-pane -t 1
tmux send-keys "printf '\033]2;tree-view\033\\'" ENTER 
tmux send-keys "clear" ENTER
tmux send-keys "cd ~/tmp; tree" ENTER
tmux att
```