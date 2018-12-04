---
layout: post
title: Use Jinja to create gazebo sdf files
categories: gazebo
tags: [vscode, gazebo, jinja, sdf]
---

## Jinja 101
Jinja2 is a template engine

### Install
~~~
pip install Jinja2
~~~

### Syntax
~~~
{{ }} Print and Evaluate an expression
{% %} Statement
{# #} Comments
~~~

### Set variable
~~~jinja
{% set title='hello world' }
{{ title }}
~~~


### Control 
- for loop
~~~bash
{% set names = ['a', 'b', 'c']%}
{% for name in names %}
    {{ loop.index }}: {{name}}
{% endfor %}

# Output result

    1: a

    2: b

    3: c
~~~

|  Variable   | Desc     |
| --- | --- | 
| loop.index    | index loop start with 1    | 
| loop.index0   | index loop 

- if statment
~~~
{% set name='' %}
{% if name %}
    {{ name }}
{% else %}
    no name enter
{% endif %}
~~~

### White space control
use + and - are control whitespace
- Run the loop example with hyphen at the end of line
~~~bash
{% set names = ['a', 'b', 'c'] -%}
{% for name in names -%}
    {{ loop.index }}: {{name}}
{% endfor -%}

# same output without line spaces
1: a
2: b
3: c
~~~

### Macro
~~~bash
{%- macro box(x, y, z) -%}
<geometry>
  <box>
    <size>{{x}} {{y}} {{z}}</size>
  </box>
</geometry>
{%- endmacro -%}

# Call the macro
{{ box(1,1,1) }}
~~~

## CLI for Jinja2
A CLI interface to Jinja2

- install
```
sudo pip install jinja2-cli
```
- usage
```
Usage: jinja2 [options] <input template> <input data>
```
### Demos
- Template
```
{% if foo is defined -%}
    foo difined and has value: define {{ foo }}
{% endif -%}
```

- Test
```bash
jinja2 -D foo=foo_value hello.txt.jinja

# result
foo difined and has value: define foo_value
```

## Tips
- Install vscode ext for better syntax color

![](/images/2018-12-04-23-16-27.png)

## Reference
- [Basics of Jinja Template Language](https://overiq.com/flask-101/basics-of-jinja-template-language/#attributes-and-method)