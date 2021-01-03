---
layout: post
title: Python logging basic
categories: python
tags: [logging]
image: logging.png
description: Using python logging with json file chat sheet
public: true
---

# Content
- Know your players
- Hello logger config
- Customize
  - color formatter

&nbsp;  
&nbsp;  
&nbsp;  
# Logger players
- Loggers
  - level
  - propagate
  - filters
  - handlers
- Handlers
  - class
  - level
  - filters
  - formatter
- Formatters
  - format
  - date format
  - style (%, {, $}) (string format)
- Filters
  - function of type: filter(recored: LogRecored) -> bool


- Logging levels
  - debug
  - info
  - warning
  - error
  - critical


# Log message flow

![](/images/pythonlogger.png)
&nbsp;  
&nbsp;  
&nbsp;  
# Hello logger config
## json config file
{% gist 8a69bfb7a2444e1ce3d592c609b314b5 %}


## config usage

```python
import json
import logging
import logging.config
import os

def main():
    logger = logging.getLogger("main")
    logger.info("info message")
    logger.debug("debug message")


if __name__ == "__main__":
    
    # Load the configuration.
    config_file = "log_json.json"
    
    with open(config_file, "r") as fd:
        config = json.load(fd)

    logging.config.dictConfig(config["logging"])
    main()
    logger = logging.getLogger()
    logger.debug("root debug message")
    logger.info("root info message")
    logger.warning("warning message")
    logger.error("error message")
    logger.critical("critical message")
```

# Customize
## color formatter
- formatter class
- config settings
- usage
  
### custom formatter class
```python
import logging

RESET_SEQ = "\033[0m\t"
COLOR_SEQ = "\033[1;%dm"

RED = 31
GREEN = 32
YELLOW = 33
BLUE = 34
WHITE  = 97

COLORS = {
    'DEBUG': BLUE,
    'INFO': GREEN,
    'WARNING': YELLOW,
    'ERROR': RED,
    'CRITICAL': RED
}

class ColorFormatter(logging.Formatter):
    def __init__(self, msg,x, y):
        logging.Formatter.__init__(self, msg,x, y)

    def format(self, record):
        levelname = record.levelname
        log_color = COLOR_SEQ % (COLORS[levelname])

        record.msg = log_color + record.msg + RESET_SEQ
        record.levelname = log_color + levelname + RESET_SEQ
        return logging.Formatter.format(self, record)
```

### config
- add formatter
> class: full path to class definition

```json
"formatters": {
    "color": {
        "class": "color_formatter.ColorFormatter",
        "style": "{",
        "datefmt": "%I:%M:%S",
        "format": "{levelname:8s}; {asctime:s}; {name:<15s} {lineno:4d}; {message:s}"
    }

    ... other formatters
}
```

### usage
- Run the above python code

![](/images/2019-10-25-10-11-37.png)


# Reference
- [bash color and formatting](https://misc.flogisoft.com/bash/tip_colors_and_formatting)