---
layout: post
title: PlantUML
categories: vscode
tags: [uml]
description: PlatUML It is an open source tool that allows you to define UML diagrams with plain text, This post show how to integrated with vscode and markdown
image: plantuml.png
public: true
---
# Content
- Install
- Basic usage
  - markdown
  - Examples
- Github / Git-pages integration
- Reference

# Install
![](/images/2019-06-09-09-01-50.png)

PlantUML has two render method
- Local: For local render `java` and `Graphviz` need to be installed
```
sudo apt install default-jre graphviz
```
- Server: Using server to render and return 

# Basic usage
## Markdown
- Activity diagram
```plantuml
@startuml
(*) --> "Initialization"

if "Some Test" then
  -->[true] "Some Activity"
  --> "Another activity"
  -right-> (*)
else
  ->[false] "Something else"
  -->[Ending process] (*)
endif

@enduml
```

## Examples
Diagrams from [real-world-plantuml](https://real-world-plantuml.com/)

- [sequence](https://real-world-plantuml.com/?type=sequence)
- [usecase](https://real-world-plantuml.com/?type=usecase)
- [class](https://real-world-plantuml.com/?type=class)
- [activity](https://real-world-plantuml.com/?type=activity)
- [component](https://real-world-plantuml.com/?type=component)

# Github / Git-pages 
- image from `blog.anoff.io`
  
![](/images/2019-06-09-21-09-45.png)

Planuml server proxy
- src: url to diagram file 
```
![cached image](http://www.plantuml.com/plantuml/proxy?src=https://raw.github.com/plantuml/plantuml-server/master/src/main/webapp/resource/test2diagrams.txt]
```
# Reference
- [Real world PlantUML](https://real-world-plantuml.com/)
- [Markdown native diagrams with PlantUML](https://blog.anoff.io/2018-07-31-diagrams-with-plantuml/)