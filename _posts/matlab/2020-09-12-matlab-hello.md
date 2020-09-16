---
layout: post
title: Matlab python integration
categories: matlab
tags: [python]
description:
public: true
---

# init and info commands
## ver
- check version and installed toolbox

```
ver
```

## Path
- Search folders
  - GUI: ENVIRONMENT (toolbar) -> Set Path

- Default path

```
# display
userpath()
# set
userpath("<Path>")
```

## Save
- Save commands from commands history
- Save Workspace: Home(toolbar) -> save workspace


## Command and help
```
>> clc

>> help clc
 clc    Clear command window.
    clc clears the command window and homes the cursor.
```

## command window 
> suppress output with `;`
### format
```
format compact
format hex
```


# Matlab image processing
![](/images/2020-09-12-19-04-00.png)

## webcam addon
[webcam addon](https://www.mathworks.com/matlabcentral/fileexchange/45182-matlab-support-package-for-usb-webcams)
![](/images/2020-09-12-20-21-14.png)