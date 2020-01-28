---
layout: post
title: Hello espressif ESP32-MDF
categories: esp
tags: [hw, esp32, mdf]
public: true
description: ESP32 Hello , install tool chain setup dev environment
image: esp32.jpeg
---


### Get ESP-IDF
```bash
cd ~/esp
git clone --recursive https://github.com/espressif/esp-mdf.git
```

### Setup
- Add IDF_PATH to  `.profile` file
```bash
export MDF_PATH=$HOME/esp/esp-mdf
```

### Get started example
- Root node
- None-root node

#### menuconfig
From `Example Configuration`
![](../../images/5c9af743.png)

- make
```bash
make erase_flash flash -j5  ESPBAUD=921600 ESPPORT=/dev/ttyUSB0
```

- monitor
```bash
make monitor ESPBAUD=921600 ESPPORT=/dev/ttyUSB0
```

## References
- [ESP-MDF Programing Guide](https://docs.espressif.com/projects/esp-mdf/en/latest/)
- [ESP-MESH](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/mesh.html)
- [get-started](https://github.com/espressif/esp-mdf/tree/master/examples/get-started)
- [Exploring WiFi Mesh with the ESP32](https://locomuco.github.io/esp32-tutorial/tutorial/index.html#0)
- [ESP32-DevKitC-32D](https://www.mouser.co.il/ProductDetail/Espressif-Systems/ESP32-DevKitC-32D?qs=sGAEpiMZZMu3sxpa5v1qrhdb%2FEQy4dLJZGgRVZsoBQk%3D)
- [Esp-prog](https://www.mouser.co.il/ProductDetail/Espressif-Systems/ESP-PROG?qs=sGAEpiMZZMu3sxpa5v1qrlA79K9tE%252BLcCKGQ1tMe4O0%3D)
- [VSCode ESP32](https://github.com/VirgiliaBeatrice/esp32-devenv-vscode/blob/master/tutorial.md)