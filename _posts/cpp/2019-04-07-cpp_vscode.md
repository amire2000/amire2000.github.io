---
layout: post
title: CPP VSCode
categories: cpp
tags: [cpp, vscode]
---
Config tasks.json and launch.json to build run and debug current file

- Project structure
- Tasks
- launch
- keybinding
- VSCode variables
    - predefined
    - environments
    - config
  
# Project struct
- bin
- src
- tests
  
# Tasks
Config tasks.json and launch.json to build run and debug current file

- tasks.json
    - Build task
    - Run task
  
## Build task
-  g: debugger symbols
-  o: folder output

```json
{
    "label": "build",
    "type": "shell",
    "command": "g++",
    "args": [
        "-g", 
        "${file}",
        "--std",
        "c++17",
        "-o",
        "bin/${fileBasenameNoExtension}"
    ],
    "group": {
        "kind": "build",
        "isDefault": true
    }
}
```

## Run task
```json
{
    "label": "Run",
    "type": "shell",
    "command": "bin/${fileBasenameNoExtension}"
}
```

# launch
Modify
- `program` key
```json
{
    "name": "(gdb) Launch",
    "type": "cppdbg",
    "request": "launch",
    "program": "${workspaceFolder}/bin/${fileBasenameNoExtension}",
    "args": [],
    "stopAtEntry": false,
    "cwd": "${workspaceFolder}",
    "environment": [],
            "externalConsole": true,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
}
```

# keybinding
Modify user keybindings.json
```json
{
    "key": "ctrl+b",
    "command": "workbench.action.tasks.runTask",
    "args": "Run",
    "when": "editorTextFocus"
}
```
# VSCode variables
## Predefined
    - ${workspaceFolder} - the path of the folder opened in VS Code
    - ${file} - the current opened file
    - ${fileBasename} - the current opened file's basename
    - ${fileBasenameNoExtension} - the current opened file's basename with no file extension
    -  ${cwd} - the task runner's current working directory on startup
> Use intellisense to browse all variables `${}`

# Reference
- [Variables Reference](https://code.visualstudio.com/docs/editor/variables-reference)
- [Jason Turner **](https://www.youtube.com/user/lefticus1/playlists)
