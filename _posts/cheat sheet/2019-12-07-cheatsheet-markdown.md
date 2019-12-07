---
layout: post
title: Markdown tips and recipes
categories: cheat
tags: [git]
public: true
image: markdown.png
---

# Control Image Size
- Add css style to markdown 
- Add `#` with style name to image path

```css
<style>
img[src*='#size1'] {
    width: 400px;
    height: 200px;
}
</style>
```

```
![](/images/xxx.png#size1)
```