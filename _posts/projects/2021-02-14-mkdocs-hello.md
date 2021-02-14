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

## Mermaid Graphs
Mermaid lets you represent diagrams using text and code.
[Mermaid](https://mermaid-js.github.io/mermaid/#/)

### code example
Add code into `mermaid` section
```
graph LR
    Hello --> MermaidPlugin
```


```mermaid
graph LR
    Hello --> MermaidPlugin
```

Install the plugin:
```
pip install mkdocs-mermaid2-plugin
```

Enter to https://www.cdnpkg.com/mermaid/file/mermaid.min.js/ and download the file mermaid.min.js (version 8.6.3)

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