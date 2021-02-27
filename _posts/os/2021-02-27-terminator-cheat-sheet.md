---
layout: post
title: Terminator cheat-sheet
categories: os
tags: [terminator]
description: terminator config, usage and scripting example
public: true
image: terminator.png
---

# Set default terminal (emulator)

```bash
sudo update-alternatives --config x-terminal-emulator

  Selection    Path                             Priority   Status
------------------------------------------------------------
  0            /usr/bin/terminator               50        auto mode
* 1            /usr/bin/gnome-terminal.wrapper   40        manual mode
  2            /usr/bin/terminator               50        manual mode
```

# layout

- Run `terminator` split and config layout
- Right click on any terminal are select `preference`
- Add and Save
  - Adding take current layout
- Create `~/.config/terminator/config` file with all terminator configuration
  
![](/images/2021-02-27-23-46-52.png)


## load terminator with layout

```bash
terminator -l <layout name>
```

## Create shortcut
- Create a shortcut with config terminator settings

> binary: `/usr/bin/terminator` 
> icons: `/usr/share/icons/hicolor/48x48/apps/terminator.png`

- Create `terminator-quad.desktop` at `~/Desktop` folder


```
#!/usr/bin/env xdg-open
[Desktop Entry]
Version=1.0
Type=Application
Terminal=false
Exec=/usr/bin/terminator -l quad
Name=Quad
Comment=Quad
Icon=/usr/share/icons/hicolor/48x48/apps/terminator.png
```

- Minimize all
- Right click on `terminator-quad.desktop` and select `Allow Launching`

![](/images/2021-02-28-00-18-06.png)

# keybinding

|              |                    |
| ------------ | ------------------ |
| ctrl-0       | Reset Font size    |
| ctrl-shift + | Increase Font size |
| ctrl -       | Decrease Font size |

|     |     |
| --- | --- |
| Move to the terminal above the current one: | Alt + ↑ |
| Move to the terminal below the current one: | Alt + ↓ |
| Move to the terminal left of the current one | Alt + ← |
| Move to the terminal right of the current one | Alt + → |
| ctrl + tab | Move to next terminal in open window |

|                     |                              |
| ------------------- | ---------------------------- |
| Ctrl + Shift + O    | Split terminals horizontally |
| Ctrl + Shift + E    | Split terminals              |
| Close current Panel | Ctrl + Shift + W             |
| Ctrl + Shift + z    | Zoom current terminal        |

|                        |                         |
| ---------------------- | ----------------------- |
| Ctrl + Shift + ↑ ↓ ← → | Resize current terminal |

|         |                         |
| ------- | ----------------------- |
| Alt + A | Broadcast all terminals |
| Alt + O | Broadcast off           |
