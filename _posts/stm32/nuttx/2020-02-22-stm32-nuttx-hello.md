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
- Debug
- Write first application
&nbsp;  
&nbsp;  
- HW: nucleo-f446re

&nbsp;  
&nbsp;  
&nbsp;  
# Install nuttx on nucleo board
- cloen openocd
- clone nuttx apps and tools

### openocd
```bash
mkdir ~/nuttxspace
cdd ~/nuttxspace
git clone http://repo.or.cz/r/openocd.git
#TODO: what to enable for stlink support only
./configure --enable-internal-jimtcl --enable-maintainer-mode --disable-werror --disable-shared --enable-stlink --enable-jlink --enable-rlink --enable-vslink --enable-ti-icdi --enable-remote-bitbang
#
make
sudo make install
```

### nuttx
```bash
ccd ~/nuttxspace
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

## nuttx space
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
- find config for board
- select board

```bash
find . -name *f446*
#
./boards/arm/stm32/nucleo-f446re
```

```
cd nuttx/tools
./configure.sh nucleo-f446re/nsh
```
> make menuconfig
> - Build setup -> Check for Build host Platform (Linux)
> - System type -> Toolchain Selection

![](/images/2020-02-23-00-51-20.png)

- check for toolchain
- check for correct ARM mcu selecction
- prepared build for debug
  - Build Setup -> Debug option ->  `Generate Debug Symbols `
  - Build Setup -> Optimization Level -> `Suppress Optimization`
  
![](/images/2020-02-23-00-48-21.png)


### make
```
make -j4
```

## flush
- find target config

```bash
cd ~/nuttxspace/openocd
find . -name *f4*.cfg
#
./tcl/target/stm32f4x.cfg
./tcl/target/stm32f4x_stlink.cfg
```

### Check

```bash
openocd \
-f interface/stlink.cfg  \
-f target/stm32f4x.cfg 
#result
Open On-Chip Debugger 0.10.0+dev-01000-gdb23c13d (2020-01-05-19:27)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : clock speed 2000 kHz
```
## Tip
- If we fail to connect with openocd and get errors like

```
Error: jtag status contains invalid mode value - communication failure
Polling target stm32f4x.cpu failed, trying to reexamine
Examination failed, GDB will be halted. Polling again in 100ms
Info : Previous state query failed, trying to reconnect
```

- shutdown openocd
- press and hold device `reset` button
- run openocd again
- release `reset` button

&nbsp;  
&nbsp;  
&nbsp;  
### run flashing
```
openocd \
-f interface/stlink.cfg  \
-f target/stm32f4x.cfg \
-c init \
-c "reset halt" \
-c "flash write_image erase nuttx.bin 0x08000000"
```

## Get nsh
> Reset board to get prompt


```
miniterm.py /dev/ttyACM0 115200
--- Miniterm on /dev/ttyACM0  115200,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---

NuttShell (NSH) NuttX-8.2
nsh> 
```
&nbsp;  
&nbsp;  
&nbsp;  
# Debug
- Run openocd
- Run GDB

## openocd
```bash
openocd \
-f interface/stlink.cfg  \
-f target/stm32f4x.cfg \
-c init \
-c "reset halt" 
#
Info : clock speed 2000 kHz
Info : STLINK V2J35M26 (API v2) VID:PID 0483:374B
Info : Target voltage: 3.258039
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : Listening on port 3333 for gdb connections
...
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections

```

## gdb
```bash
cd ~/nuttx/workspace
cd nuttx
arm-none-eabi-gdb nuttx
#
(gdb) target remote :3333
Remote debugging using :3333
__start () at chip/stm32_start.c:271
271	{
#
(gdb) info threads 
  Id   Target Id         Frame 
* 1    Remote target ""  __start () at chip/stm32_start.c:271
#
(gdb) info registers 
r0             0x0	0
r1             0x0	0
r2             0x0	0
r3             0x0	0
r4             0x0	0
r5             0x0	0
r6             0x0	0
r7             0x0	0
r8             0x0	0
r9             0x0	0
r10            0x0	0
r11            0x0	0
r12            0x0	0
sp             0x20001dac	0x20001dac
lr             0xffffffff	-1
pc             0x80001c4	0x80001c4 <__start>
xPSR           0x1000000	16777216

```
## debug with vscode
- Install `Cortex-Debug` extension
- Open nuttx folder with `vscode`
  - Add setting to `launch.json`

### SVD file
- [STM32 svd files](https://github.com/posborne/cmsis-svd/tree/master/data/STMicro)

### launch file
- toolchain path
- svd file path
- config files


```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Cortex Debug",
            "cwd": "${workspaceRoot}",
            "executable": "nuttx",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "openocd",
            "device": "stm32f4x",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32f4x.cfg"
            ],
            "armToolchainPath": "~/toolchains/gcc-arm/bin/",
            "svdFile": "~/svd/STM32F446.svd"
        }
    ]
}
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
- [SVD files](https://github.com/posborne/cmsis-svd/tree/master/data/STMicro)
- [Nuttx tutorial](https://youtu.be/heSkSd-_70g?list=PLd73yQk5Fd8JEsVD-lhwYRQKVu6glfDa8)
- [My first Nuttx Application](http://nuttx.nl/index.php/2018/03/30/my-first-nuttx-application/)
- [First app](https://hmchung.gitbooks.io/stm32-tutorials/write-an-application-on-nuttx.html)
- [gdb openocd](https://micro-ros.github.io/docs/tutorials/advanced/debugging_gdb_openocd/)