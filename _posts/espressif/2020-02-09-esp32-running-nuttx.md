---
layout: post
title: Running NuttX on ESP32 board
categories: hw
tags: [nuttx, esp32]
public: true
description: 
image: espressif.png
---

# LAB
- Install esp-idf on ubuntu machine
- Create and upload hello-world example for bootloader
  - fix bootloader watchdog for future error
- Build nuttx for esp32
- Upload/ Flush
  - prepared files and bootloader
- connect

# Folders

```
├── esp
│   ├── esp-idf
│   ├── hello_world
│   └── xtensa-esp32-elf
└── nuttxspace
    ├── apps
    ├── nuttx
    └── tools
```

# Fix bootloader
- From esp `hello-world` folder
- Run `make menuconfig`
- Select `bootloader config--->` menu
- Remove `Use RTC watchdog in start code` if selected
- Run `make`
  
![](/images/2020-02-09-06-41-14.png)

# Build
```
cd nuttxspace/nuttx
make distclean
./tools/configure.sh esp32-core/nsh
make
```

# Prepared
> esptool.py locate at `/esp-idf/components/esptool_py/esptool/esptool.py`

- generated `nuttx.bin` with `esptool`
```
esptool.py --chip esp32 elf2image --flash_mode dio --flash_size 4MB -o ./nuttx.bin nuttx
```

![](/images/2020-02-09-07-00-11.png)

![](/images/2020-02-09-07-01-17.png)

## copy bootloader from idf folder
```
cd ~/esp/hello_world
cp build/bootloader/bootloader.bin ~/nuttxspace/nuttx/
cp build/partitions_singleapp.bin ~/nuttxspace/nuttx/partitions.bin
```

## Upload
```
sudo esptool.py --chip esp32 --port /dev/ttyUSB0 \
--baud 921600 \
write_flash \
0x1000 bootloader.bin \
0x8000 partitions.bin \
0x10000 nuttx.bin
```
- 0x1000: Second stage bootloader
- 0x8000: Second stage bootloader reads the partition table found at offset 
# Connect

```
miniterm.py /dev/ttyUSB0 115200
```

![](/images/2020-02-09-07-08-51.png)

# Reference 
- [Running NuttX on ESP32 board](https://acassis.wordpress.com/2018/01/04/running-nuttx-on-esp32-board/)
- [ESP32 guide general](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/general-notes.html)