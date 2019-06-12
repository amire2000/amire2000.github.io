---
layout: post
title: Learning ArduPilot code part 1
categories: APM
tags: [ardupilot]
public: true
image: apm.png
description: Clone and setup ardupilot code with VSCode IDE, View basic ardupilot code structure, Run and attach debugger, Use APMRover2 as demo.
---
# Content
- First Run
- ArduPilot Code Structure
- VSCode config
- Config VSCode debugger

# ArduPilot Code Structure
- Vehicle code: Specific code for each vehicle type: rover, copter ....
- Shared libraries: Shared amongst the vehicle, include sensor drivers, position estimation and control code
- Hardware abstraction layer HAL

# VSCode

![](/images/2019-06-12-07-49-40.png)
- AP has a large code base
- Using `files.watcherExclude` to remove `static` folder from watcher

`.vscode/settings.json`

```json
"files.watcherExclude": {
    "**/.github/**": true,
    "**/build/**": true,
    "**/benchmarks/**": true,
    "**/docs/**": true,
    "**/mk/**": true,
    "**/modules/**": true,
    "**/tests/**": true,
    "**/Tools/**": true
}
```
# Config VSCode debugger
- Compile with debug symbols
- Add `Attach` task to launch.json
- Run SITL with `-D` flag
- Attach VSCode to running process
- Debug :)
  
## Compile (SITL) with debug symbols
```
./waf configure --debug
```

## Add (gdb) Attach section to launch.json
![](/images/2019-06-12-08-09-13.png)
- set `program` to sitl ardurover
  - `"program": "${workspaceFolder}/build/sitl/bin/ardurover"`
  

## Launch SITL with D debug flag
```
sim_vehicle.py -v APMrover2 -f rover -D
```

## Start debugging by attach to process
- Run launch
- Select process to attach 
- VSCode ask for permission ([check ptrach permission](http://ardupilot.org/dev/docs/debugging-with-gdb-on-linux.html))
- Set breakpoints


![](/images/2019-06-12-08-25-10.png)
# Reference
- [Learning ArduPilot — Introduction](http://ardupilot.org/dev/docs/learning-ardupilot-introduction.html)
- [Debugging with GDB on linux](http://ardupilot.org/dev/docs/debugging-with-gdb-on-linux.html)
