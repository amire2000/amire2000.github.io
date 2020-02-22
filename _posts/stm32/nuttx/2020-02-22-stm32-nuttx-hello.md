---
layout: post
title: Nuttx on STM32 Nucleo board hello
categories: hw
tags: [nuttx, nucleo, stm32]
public: true
description: Flush nuttx on stm32 nucleo F446RE board, write first application
image: nuttx.png
---
# lab
- Install nuttx on nucleo board
- Write first application
&nbsp;  
&nbsp;  
- HW: nucleo-f446re

&nbsp;  
&nbsp;  
&nbsp;  
# Init nuttx space
- download from git
- Config board
- Make
```bash
mkdir nuttxspace
git clone https://bitbucket.org/nuttx/nuttx.git nuttx
git clone https://bitbucket.org/nuttx/apps.git apps
git clone https://bitbucket.org/nuttx/tools.git tools
#
cd tools/kconfig-frontends/
./configure
make
sudo make install
sudo ldconfig
```
> make menuconfig
> - Build setup -> Check for Build host Platform (Linux)
> - System type -> Toolchain Selection

![](/images/2020-02-23-00-51-20.png)

![](/images/2020-02-23-00-48-21.png)

```
├── nuttxspace
       ├── apps
       ├── nuttx
       ├── openocd
       └── tools
```
&nbsp;  
&nbsp;  
&nbsp;  
## Prepared config
- select board
  
```
cd nuttx/tools
./configure.sh nucleo-f446re/nsh
```
&nbsp;  
&nbsp;  
&nbsp;  
## first app
```
├── apps
    ├── example
        ├── my_app
               ├── Kconfig
               ├── Make.defs
               ├── counter.c
               └── Makefile
```
### Create app struct
```
mkdir -p apps/examples/my_app
cd apps/examples/my_app
touch Kconfig Make.defs counter.c Makefile
```

### Kconfig
```
config EXAMPLES_MY_APP
bool "MY FIRST APP - HELLO"
default n
---help---
Enable the \"Hello\" Example
if EXAMPLES_MY_APP
endif
```

### Make.defs
```
ifeq ($(CONFIG_EXAMPLES_MY_APP), y)
CONFIGURED_APPS += examples/my_app
endif
```

### Makefile
```
-include $(TOPDIR)/Make.defs

APPNAME = counter
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = 2048

ASRCS =
CSRCS =
MAINSRC = counter.c

CONFIG_MY_APP_PROGNAME ?= counter$(EXEEXT)
PROGNAME = $(CONFIG_MY_APP_PROGNAME)

include $(APPDIR)/Application.mk
```

### example folder Kconfig
- Add line to `apps/examples/Kconfig`
  
```
source "$APPSDIR/examples/my_app/Kconfig"
```
### counter.c
```c
#include <nuttx/config.h>
#include <stdio.h>
#ifdef CONFIG_BUILD_KERNEL
int main (int argc, FAR char *argv[])
#else
int counter_main(int argc, char *argv[])
#endif
{
    printf("Hello world! Here I go... \n");
    return 0;
}
```
## menuconfig
- From `nuttx` folder run `make menuconfig`
- Select `Application Configuration -> Examples` menu

![](/images/2020-02-23-00-34-05.png)

- Exit

## make
```
make -j4
```

## flush
```bash
openocd -f interface/stlink-v2-1.cfg  \
-f target/stm32f4x.cfg \
-c init \
-c "reset halt" \
-c "flash write_image erase nuttx.bin 0x08000000"
```

```bash
miniterm.py /dev/ttyACM0 115200
# or
minicom -D /dev/ttyACM0
```
## run
- Press hard reset on board

```
NuttShell (NSH) NuttX-8.2
nsh> ␛[K?
help usage:  help [-v] [<cmd>]

  [         cd        echo      hexdump   mkrd      pwd       sleep     umount    
  ?         cp        exec      kill      mh        rm        test      unset     
  basename  cmp       exit      ls        mount     rmdir     time      usleep    
  break     dirname   false     mb        mv        set       true      xd        
  cat       dd        help      mkdir     mw        sh        uname     

Builtin Apps:
  counter  ostest   hello    nsh      
nsh> counter
Hello world!  Here I go... 
```

&nbsp;  
&nbsp;  
&nbsp;  
# Resource
- [My first Nuttx Application](http://nuttx.nl/index.php/2018/03/30/my-first-nuttx-application/)
- [First app](https://hmchung.gitbooks.io/stm32-tutorials/write-an-application-on-nuttx.html)