---
layout: post
title: JSON and YAML schema
categories: VSCode
tags: [vscode, yaml, json]
---
- JSON Schema validation 101
- VSCode settings
- YAML Schema support
- Example (catmux yaml)
- Reference
  
# VSCode settings
- settings.json
  > schema file locate from project root

```json
"json.schemas": [
        {
            "fileMatch":[
                "*.json",
                "catmux.yaml"
            ],
            "url":"./catmux.schema.json"
        }
    ],
    "yaml.schemas": {
        "./catmux.schema.json": "catmux.yaml"
    }
```
# YAML Schema support
- Redhat yaml extension support json schema for YAML
![](/images/2019-04-28-16-30-03.png)

# Example (catmux yaml)
- usage
![](/images/2019-04-28-16-36-57.png)

- schema file (`<project_root>/catmux.shema.json`)
```json
{
    "$id": "https://ros.com/catmux.schema.json",
    "$schema": "http://json-schema.org/draft-07/schema",
    "title": "catmux",
    "description": "catmux ros tmux",
    "type": "object",
    "properties": {
        "common": {
            "type": "object",
            "properties": {
                "before_commands": {
                    "type": "array",
                    "items": {
                        "type": "string",
                        "minimum": 0
                    }
                },
                "default_window": {
                    "type": "string"
                }
            }
        },
        "parameters": {
            "type": "object",
            "properties": {
                "show_layouts": {
                    "type": "boolean",
                    "default": false
                },
                "replacement_param": {
                    "type": "string"
                }
            }
        },
        "windows": {
            "type": "array",
            "items": {"$ref": "#/definitions/win_item"}
        }
    },
    "definitions": {
        "win_item": {
            "type": "object",
                "properties": {
                    "name": {
                        "type": "string"
                    },
                    "layout": {
                        "type": "string",
                        "oneOf": [
                            {"enum": ["even-horizontal", "even-vertical", "tiled"]}
                        ]
                    },
                    "splits": {
                        "type": "array",
                        "items": {"$ref": "#/definitions/win_commands"}
                    }
                }
        },
        "win_commands": {
            "type": "object",
            "properties": {
                "commands": {
                    "type": "array",
                    "items": {
                        "type": "string"
                    }
                }
            }
        }
    }
}
```

# Reference
- [Getting Started Step-By-Step](http://json-schema.org/learn/getting-started-step-by-step.html#intro)
- [Miscellaneous Examples](http://json-schema.org/learn/miscellaneous-examples.html)