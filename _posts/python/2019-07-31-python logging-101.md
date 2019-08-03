---
layout: post
title: Python logging basic
categories: python
tags: [python, logging]
image: logging.png
description: Using python logging with json file chat sheet
public: true
---

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
# json config file
{% gist 8a69bfb7a2444e1ce3d592c609b314b5 %}


# usage
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
    logger.info("root info message")
    logger.debug("root debug message")
```