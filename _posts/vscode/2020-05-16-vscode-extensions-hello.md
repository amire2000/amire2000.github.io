---
layout: post
title: VSCode and Docker as dev environment
categories: vscode
tags: [vscode, ext]
description: VSCode extension hello
image: 
public: true
---
## install
```
sudo npm install -g generator-code
```

## Create
```
yo code
```

![](/images/2020-05-16-22-30-09.png)


## Running and debugging
```
code --extensionDevelopmentPath=`pwd`/urdf
```

### Running
- Open two vscode's
  - extension code
  - running

> Tip: npm compile extension code on change

> Reload `ctrl+r` to load extesion after build 

&nbsp;  
&nbsp;  
&nbsp;  
# Add command
- [vscode api command](https://code.visualstudio.com/api/extension-guides/command)

## package.json
```json
{
  "contributes": {
    "commands": [
      {
        "command": "myExtension.sayHello",
        "title": "Say Hello"

      }
    ]
  }
}
```

```json
{
  "activationEvents": ["onCommand:myExtension.sayHello"]
}
```

- extension.ts

```typescript
export function activate(context: vscode.ExtensionContext) {
  const command = 'myExtension.sayHello';

  const commandHandler = (name: string = 'world') => {
    console.log(`Hello ${name}!!!`);
  };

  context.subscriptions.push(vscode.commands.registerCommand(command, commandHandler));
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# Snippets
- [vscode api](https://code.visualstudio.com/api/language-extensions/snippet-guide)
## package.json
```json
{
  "contributes": {
    "snippets": [
      {
        "language": "javascript",
        "path": "./snippets.json"
      }
    ]
  }
}
```

## Snippets short guid
- [Snippet syntax](https://code.visualstudio.com/docs/editor/userdefinedsnippets#_snippet-syntax)

- Tabstop

With tabstops, you can make the editor cursor move inside a snippet. Use $1, $2 to specify cursor locations. The number is the order in which tabstops will be visited, whereas $0 denotes the final cursor position. Multiple occurrences of the same tabstop are linked and updated in sync.


- Placeholder

```
${1:foo}
```

- Choice

```
${1|one,two,three|}
```

- Varibles

[check](https://code.visualstudio.com/docs/editor/userdefinedsnippets#_variables)

&nbsp;  
&nbsp;  
&nbsp;  
## code samples 
### Get current file path
```typescript
var path = require("path");
var currentlyOpenTabfilePath = vscode.window.activeTextEditor.document.fileName;
var currentlyOpenTabfileName = path.basename(currentlyOpenTabfilePath);
```
&nbsp;  
&nbsp;  
### Run external Process
```
const cp = require('child_process')
cp.exec('pwd', (err: string, stdout: string, stderr: ing) => {
	console.log('stdout: ' + stdout);
	console.log('stderr: ' + stderr);
	if (err) {
		console.log('error: ' + err);
	}
});
```
&nbsp;  
&nbsp;  
###
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [vscode docs](https://vscode-docs.readthedocs.io/en/stable/extensions/debugging-extensions/)
- [extensions sample](https://github.com/microsoft/vscode-extension-samples)