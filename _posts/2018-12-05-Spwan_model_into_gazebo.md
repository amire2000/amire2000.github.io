---
layout: post
title: Spawn model into gazebo
categories: gazebo
tags: [gazebo, sdf]
---

## GAZEBO ENV. Variables
`/usr/share/gazebo/setup.sh`

| Variable           | Description      |
| ------------------ | ---------------- |
| GAZEBO_PLUGIN_PATH | models locations |

## gz command
This tool modifies various aspects of a running Gazebo simulation.

  `Usage:  gz <command>`

List of commands:
```
  help      Print this help text.
  camera    Control a camera
  debug     Returns completion list for a command. Used for bash completion.
  help      Outputs information about a command
  joint     Modify properties of a joint
  log       Introspects and manipulates Gazebo log files.
  model     Modify properties of a model
  physics   Modify properties of the physics engine
  sdf       Converts between SDF versions, and provides info about SDF files
  stats     Print statistics about a running gzserver instance.
  topic     Lists information about topics on a Gazebo master
  world     Modify world properties
```