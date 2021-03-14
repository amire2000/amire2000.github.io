---
layout: post
title: Git submodules
categories: vscode
tags: [git]
public: true
image: git.png
description: Git projects and submodules 
---

# GitHub Wiki in a Repository as a Submodule 
- GitHub Wiki is itself a git repo

## setup
- from project folder
```bash
# clone repo to docs folder
git submodule add <project>.wiki.git docs
git commit -m 'first commit with submodule docs'
git push
```

## clone
- clone parent
- clone all submodules

```bash
git submodule init
git submodule update
```

# Reference
- [ncluding a GitHub Wiki in a Repository as a Submodule](https://brendancleary.com/2013/03/08/including-a-github-wiki-in-a-repository-as-a-submodule/)