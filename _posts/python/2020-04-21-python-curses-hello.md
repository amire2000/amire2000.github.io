---
layout: post
title: Console TUI using curses and python
categories: python
tags: [gui, tui, curses]
image:
public: true
description: Using curses library to build console gui
---

# Demo
> Don't forget to `refresh` screen
> Don't forget to run `curses.endwin()` at the end


```python
import curses

try:
    screen = curses.initscr()
    curses.start_color()
    curses.init_pair(1, curses.COLOR_RED, curses.COLOR_YELLOW)
    screen.addstr("RED ALERT!\n", curses.color_pair(1))
    #text format
    screen.addstr("Bold\n", curses.A_BOLD)
    screen.addstr("Highlighted\n", curses.A_STANDOUT)
    screen.addstr("Underline\n", curses.A_UNDERLINE)
    screen.refresh()
    row = 10
    col = 2
    for i in range(10):
        screen.addstr(row, col, f"Counter: {i}")
        #refrash screen
        screen.refresh()
        #sleep
        curses.napms(300)

finally:
    curses.endwin()

```

# Reference 
- [Python curses](https://docs.python.org/3/howto/curses.html)
- [Curses API](https://docs.python.org/3.6/library/curses.html#curses.napms)