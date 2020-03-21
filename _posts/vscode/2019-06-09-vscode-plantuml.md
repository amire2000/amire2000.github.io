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

# PlantUML

PlantUML has two render method
- Local: For local render `java` and `Graphviz` need to be installed
  
```
sudo apt install default-jre graphviz
```
- Server: Using server to render and return 

&nbsp;  
&nbsp;  
&nbsp;  
# VSCode
[VSCode market](https://marketplace.visualstudio.com/items?itemName=jebbs.plantuml)

![](/images/2020-03-21-14-37-23.png)

## settings.json
```bash
# Server mode
"plantuml.render": "PlantUMLServer",
"plantuml.server": "https://www.plantuml.com/plantuml",

# local
"plantuml.render": "Local",

```
&nbsp;  
&nbsp;  
&nbsp;  
# Basic usage
## Markdown
- Activity diagram for example

```
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
```(TODO: just for markdown render->without the text is no output )
```
![](/images/2019-06-09-21-47-30.png)

TBD: Render Not working for gitpages

![uncached image](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://robobe.github.io/assets/images/github.plantuml.txt]


## Examples
Diagrams from [real-world-plantuml](https://real-world-plantuml.com/)

- [sequence](https://real-world-plantuml.com/?type=sequence)
- [usecase](https://real-world-plantuml.com/?type=usecase)
- [class](https://real-world-plantuml.com/?type=class)
- http://plantuml.com/activity-diagram-beta
- [activity](https://real-world-plantuml.com/?type=activity)
- [component](https://real-world-plantuml.com/?type=component)

# Github / Git-pages 
- image from `blog.anoff.io`
  
![](/images/2019-06-09-21-09-45.png)


# Reference
- [Real world PlantUML](https://real-world-plantuml.com/)
- [Markdown native diagrams with PlantUML](https://blog.anoff.io/2018-07-31-diagrams-with-plantuml/)