---
layout: post
title: VSCode power tools ext 101
categories: vscode
tags: [extensions]
public: true
description: Using PowerTools ext to manage post blog tags list
image: powet-tools.png
---
- Install Power Tools ext.
- Add power tools button to project settings
- Add button run script
  - Using `showQuickPick` method
  - Using `activeTextEditor` vscode window property
```
.vscode/
├── my_tags.js
└── settings.json
```

## settings.json
- Add to project settings
```json
"ego.power-tools": {
        "buttons": [
            {
                "text": "click",
                "action":{
                    "script": "my_tags.js",
                    "type": "script"
                }
            }
        ]
    },
```

## my_tags.js

```js
exports.execute = async (args) => {
    const vscode = args.require('vscode');
    const editor = vscode.window.activeTextEditor;
    let insertPosition = editor.selection.active;
    let list = ['zmq', 'protobuf', 'sdf', 'plugin'].sort();
    let i = 0;
	const result = await vscode.window.showQuickPick(list).then(
        selection=>{
            if (!selection) {
                return;
              }
            editor.edit(edit => 
            {
                edit.insert(insertPosition, selection);
            })
        }
    );
};

```
# Reference
- [VScode market](https://marketplace.visualstudio.com/items?itemName=ego-digital.vscode-powertools)
- [Ext wiki](https://github.com/egodigital/vscode-powertools/wiki/Events)