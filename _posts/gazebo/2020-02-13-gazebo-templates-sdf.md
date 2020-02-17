---
layout: post
title: Using python mako template to create sdf files
categories: gazebo
tags: [python, mako, template, generator]
description: 
image: makoLogo.png
public: true
---

Gazebo site has entry about `ERB` Embedded Ruby Code templates [link](http://gazebosim.org/tutorials?tut=model_structure&cat=build_robot).

It's very good idea to have something for `sdf` like `xacro` for `urdf`
I preferred python and after few google search I decide to give a try to Mako` [link](https://www.makotemplates.org/)

Mako is an embedded Python (i.e. Python Server Page) language, 

# install
```
pip install Mako
```

# simple demo

- simple_demo.mako
  
```
<%
x=1
y=2
%>

this is x: ${x}
this is y: ${y}
```

- from command line run

```
mako-render simple_demo.mako > simple_demo.txt
```

- simple_demo.txt

```

this is x: 1
this is y: 2
```

# Connect Make and VSCode
- Run Template generator `mako-render` on file save
- Syntax highlighting for the Mako templating language.

## Mako Extension
[Marketplace](https://marketplace.visualstudio.com/items?itemName=tommorris.mako)

## Run on save
[Marketplace](https://marketplace.visualstudio.com/items?itemName=emeraldwalk.RunOnSave)
This extension allows configuring commands that get run whenever a file is saved in vscode.

- Add to `settings.json`
- Run `make-render` on files with sdf.mako ext.

> Run mako from template location, help to resolve include ref


```
"emeraldwalk.runonsave": {
        "commands": [
            {
                "match": ".sdf.mako",
                "cmd": "cd ${fileDirname} && mako-render ${fileBasename} > ${fileDirname}/${fileBasenameNoExt}"
            }
        ]
    }
```

# Mako simple tutorial
[](https://docs.makotemplates.org/en/latest/syntax.html#python-blocks)

## Python block
Within <% %>, you’re writing a regular block of Python code

## Tags
### <%include>

### <%namespace>
load file template and add namespace to it objects

- Foo as a namespace before mytag function call
- Pass caller function body and use it in calee
  
#### library.mako
```
<%def name="mytag()">
    <form>
        ${caller.body()}
    </form>
</%def>
```

#### main.mako
```
<%namespace name="foo" file="library.mako"/>

<%foo:mytag>
    a form
</%foo:mytag>
```

### <%def>
```
<%def name="myfunc(x)">
    this is myfunc, x is ${x}
</%def>

${myfunc(7)}
```
### <%block>
like `<%def>` except executes immediately

## example
### if
```
% if var:
var is defined.
% else:
var is not defined.
% endif
```

### for
% for i in range(1,4):
    ${loop.index}: ${i}
% endfor

% for i in ["a", "b", "c"]:
    ${loop.index}: ${i}
% endfor

# Demo

- sdf.mako


{% gist 977de27cb638d24594778536bdca3c16 %}

- color.mako

```xml
<%def name="color(color_name)">
<lighting>1</lighting>
<script>
    <uri>file://media/materials/scripts/gazebo.material</uri>
    <name>Gazebo/${color_name}</name>
</script>
</%def>

<%def name="red()">
${color("Red")}
</%def>

<%def name="black()">
${color("Black")}
</%def>
```

- macros.mako

```xml
<%def name="cylinder(radius, length)">
    <geometry>
        <cylinder>
            <radius>${radius}</radius>
            <length>${length}</length>
        </cylinder>
    </geometry>
</%def>

<%def name="box(width, length, height)">
	<box>
		<size>${width} ${length} ${height}</size>
	</box>
</%def>
```