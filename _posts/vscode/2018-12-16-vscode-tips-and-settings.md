---
layout: post
title: VSCode settings and tips
categories: vscode
tags: [vscode]
public: true
description: vscode settings extension tips and trouble shotting
image: vscode.png
---

# Content
- [Extensions](#extensions)
- [Settings](#settings)
- [Keybindings](#keybindings)
- [Q & A](#q--a)

&nbsp;  
&nbsp;  
&nbsp;  
# Extensions
## Better Comments

Improve your code commenting by annotating with alert, informational, TODOs, and more!

![](https://raw.githubusercontent.com/aaron-bond/better-comments/master/images/better-comments.PNG)

<!-- | Name               |                                      | Desc                              |
|--------------------|--------------------------------------|-----------------------------------|
| Paste Image        | ![](/images/2018-12-19-07-16-19.png) | install xclip `apt install xclip` |
| Code runner        | ![](/images/2018-12-19-07-07-33.png) | Run Current code                  |
| Projects Manager   | ![](/images/2018-12-19-07-17-55.png) |                                   |
| Code Spell checker | ![](/images/2018-12-19-07-08-18.png) | Spell checker                     |
| XML Tools          | ![](/images/2018-12-19-07-15-36.png) |                                   | -->


&nbsp;  
&nbsp;  
&nbsp;  
# Settings
### Hide certain file from explorer
```
"files.exclude": {
        "**/__pycache__": true
    }
```

### File Association
```json
"files.associations": {
        "*.myphp": "php"
    }
```

### Open goto definition in new TAB
```
"workbench.editor.enablePreview": false
```
&nbsp;  
&nbsp;  
&nbsp;  
# Keybindings
### Add new file under the selected working directory
```json
  {
        "key": "ctrl+n",
        "command": "explorer.newFile",
        "when": "explorerViewletFocus"
    },
    {
        "key": "ctrl+shift+n",
        "command": "explorer.newFolder",
        "when": "explorerViewletFocus"
    }
```
&nbsp;  
&nbsp;  
&nbsp;  
# Q & A
## VSCode intellisense not working

- Set Python path and extraPath if needed
```json
"python.pythonPath": "/usr/bin/python",
    "python.autoComplete.extraPaths": [
       "${workspaceFolder}/customModule"
    ]
```

## Activate virtualenv for VSCode integrated terminal
```json
"terminal.integrated.shellArgs.linux": ["-c", "source ./.env/bin/activate; bash -i"]
```

