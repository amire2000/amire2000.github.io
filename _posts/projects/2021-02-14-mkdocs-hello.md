---
layout: post
title: Using MkDocs to document project
categories: vscode
tags: [markdown]
public: true
description: Using MkDocs to generated static project's document using VSCode
image: mkdocs.png
---

# MkDocs
MkDocs is a fast, simple static site generator from `Markdown`  
The site can be host on github pages and other hosting
MkDocs include man plugin to enhance the capabilities


## Install
```bash
# In project virtual env. or global
pip install --upgrade pip
pip install wheel
pip install mkdocs
```

## Project
MkDocs build from `yaml` config file and `docs` folder crate manually or with `mkdocs new project-name`

```
├── docs
|    └── index.md
├── mkdocs.yml
├── site
├── src
└── venv
```

## Usage
MkDocs comes with a built-in dev-server that lets you preview your documentation as you work on
```bash
# from project root
mkdocs serve
# open browser at 127.0.0.1:8000

```

### Minimal site
- Basic with basic navigation
- Basic theam


```yaml
site_name: MySite
nav:
  - Home: index.md
  - about: about.md
```

### basic theam
![](/images/2021-02-14-12-53-00.png)


## MkDocs command and yaml config
### Commands
- serve: Start live docs server
- build: Build site for deploy (Create a `site` folder)

### yaml
[Configuration](https://www.mkdocs.org/user-guide/configuration/)

### Plugins
#### Install
using pip 
```bash
# pip install mkdocs-foo-plugin
pip install mkdocs-bootstrap-tables-plugin
pip install mkdocs-mermaid2-plugin
pip install mkdocstrings
pip install mkdocs-build-plantuml-plugin
```
#### config
Add plugins section to `yaml` config file

Example with 
```yaml
site_name: MySite
nav:
  - Home: index.md
  - about: about.md

plugins:
  - search
  - bootstrap-tables
  - mermaid2
  - mkdocstrings


extra_javascript:
    - mermaid.min.js


```
&nbsp;  
&nbsp;  
&nbsp;  
## Plugins
### PlantUML
[PlantUML](https://plantuml.com/) is a component that allows to quickly write : 
- UML Diagrams
- Network Diagrams
- And more just [check](https://plantuml.com/)

#### VSCode extensions
![](/images/2021-02-19-12-42-11.png)

Using Alt-d to render diagrams

##### Offline settings
Change vs settings
- [check](https://marketplace.visualstudio.com/items?itemName=jebbs.plantuml)


```json
"plantuml.render": "Local"

plantuml.jar: Alternate plantuml.jar location. Leave it blank to use integrated jar.
plantuml.jarArgs:
```

#### Run PlantUML local / offline
[Local installation](https://plantuml.com/starting)
To run locally we need to install 
- java
- Graphviz: `sudo apt install graphviz`
- Download [plantuml.jar](http://sourceforge.net/projects/plantuml/files/plantuml.jar/download)
- And test

```bash
# Check for help
java -jar plantuml.jar -h

# Example
java -jar plantuml.jar -t<output type> -o<outfolder> <diagram.puml>

#-t
-tpng
-tsvg
-tpdf

#-o
```
&nbsp;  
&nbsp;  
&nbsp;  
#### MkDocs
[Github](https://github.com/christo-ph/mkdocs_build_plantuml)

```
docs/                         # the default MkDocs docs_dir directory
  diagrams/
    include/                  # for include files like theme.puml etc (optional, won't be generated)
    out/                      # the generated images, which can be included in your md files
    src/                      # the Plantuml sources
      subdir1/file1.puml
      subdir2/file.puml
mkdocs.yml 
```

##### work offline (yaml config)
```
- build_plantuml:
      render: "local"
      bin_path: "java -jar /usr/local/bin/plantuml/plantuml.jar"
```

&nbsp;  
&nbsp;  
&nbsp;  
### Tables
Render Tables from MarkDown file, create a table as:

| First Column  | Second Column  | Third Column  |
|---------------|:--------------:|---------------|
| Ex1           | Ex2            | Ex3           |



- In order to center the content, use :---: as shown in Second Column

Install the plugin:
```
pip install mkdocs-bootstrap-tables-plugin
```

Add the plugin to mkdocs.yml file:
```
plugins:
    - search
    - bootstrap-tables
```
&nbsp;  
&nbsp;  
&nbsp;  
## Mermaid Graphs
Mermaid lets you represent diagrams using text and code.
[Mermaid](https://mermaid-js.github.io/mermaid/#/)

### code example
Add code into `mermaid` section

```mermaid
graph LR
    Hello --> MermaidPlugin
```

![](/images/2021-02-14-13-30-29.png)

Install the plugin:
```
pip install mkdocs-mermaid2-plugin
```

[download mermaid.min.js](https://www.cdnpkg.com/mermaid/file/mermaid.min.js/)  (version 8.6.3)

Save the file on project/docs folder.

- Add the plugin to mkdocs.yml file:
```
plugins:
    - search
    - mermaid2
```

- Add the Mermaid library declaration to mkdocs.yml file:
```
extra_javascript:
    - mermaid.min.js
```

- Download from vscode MarketPlace the `Markdown Preview Mermaid Support` in order to see mermaid graphs on markdown file if needed.


&nbsp;  
&nbsp;  
&nbsp;  
### Code Reference

Install the plugin:
```
pip install mkdocstrings
```

- Add the plugin to mkdocs.yml file:
```
plugins:
    - search
    - mkdocstrings
```

- Add a reference in your MarkDown file:


```
::: <folder>.<file_name>.<class/function>
```
For example:

::: src.main.func

&nbsp;
&nbsp;  
&nbsp;  
&nbsp;  
# References
- [MkDocs](https://www.mkdocs.org/)