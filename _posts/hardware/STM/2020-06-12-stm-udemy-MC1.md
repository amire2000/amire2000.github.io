---
layout: post
title: MC1
categories: [hw, stm32]
tags: [stm32, udemy]
public: true
image: stm32icon.png
description: MC1 
---

Ubuntu pre setup

```bash
# For run gdb
sudo apt-get install libncurses5
```

## Hello world
### First code
```c
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <stdio.h>

int main(void)
{
	printf("hello world");
	for(;;);
}
```

### Debugger
> Don't forget to upgrade st-link firmware

```

```

### SWD Serial Wire Debug
- Serial Wire Output
