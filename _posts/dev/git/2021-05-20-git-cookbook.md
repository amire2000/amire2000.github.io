---
layout: post
title: git recipes  and vscode integration
categories: vscode
tags: [git]
public: true
image: git.png
description: git recipes, how to, cheat sheet and vscode integration
---

# LAB

Basic usage of git from command line and vscode

- [setup](#setup)
- history: files and commits
- [commit / commit message struct](#commit-message)
- [branch](#branch)
- [tags](#tag)
- [tips](#tips)

&nbsp;  
&nbsp;  
# setup

- create folder
- run `git init`
- add `src` sub folder
- add two files under src
- create first commit

```bash
cd src/
echo line_1 > a.txt
echo line_1 > b.txt
```

```
.
└── src
    ├── a.txt
    └── b.txt
```

### Create first commit

- Add files
- Commit

```bash
git status
git add .
git commit -m "commit1"
```

# History

- view commit history: `git log`

```bash
git log --oneline
#
a78fcd6 (HEAD -> master) commit1
```

- view commit files: `git show`

```bash
git show --name-only --pretty="" a78fcd6

#
src/a.txt
src/b.txt
```

### View file history

```
git log --oneline <path to file>
git log -p <path to file>
git blame <path to file>
```

### View Changes

- With working file

```
git log --oneline a.txt
c4afca6 (HEAD, master) commit3
d3a1727 commit2
a78fcd6 commit1

git show c4afca6
```

### Diff between commits

```
git diff 85400a9..a78fcd6 -- src/a.txt
```

## VSCode Timeline

![](/images/2021-05-21-06-11-37.png)

![](/images/2021-05-21-06-12-33.png)

&nbsp;  
&nbsp;  
&nbsp;

# Branch

- Crate new branch
- update `a.txt`
- Add new file `c.txt`

## view changes files

```bash
# View changes files between current branch to master
git diff --name-status master
# or
# git diff --stat --color master
#
M       src/a.txt
A       src/c.txt
```

## Diff file between branches

```bash
git diff master..branch1 -- src/a.txt
# Or current to master
git diff master -- src/a.txt
```

## VSCode Extensions

- GitLens
- Git Graph

### View file diff between two commits

- open request file in editor
- goto `Gitlence` command pallet: `Open File History`

&nbsp;

- Select file commit to Compare
  ![](/images/2021-05-21-08-41-23.png)

- Compare with selected
  ![](/images/2021-05-21-08-44-06.png)

# View branch with Git Graph

![](/images/2021-05-21-13-26-37.png)

# Commit message

```
<type>: <description>

[optional body]

[optional footer]
```

|          |                                                             |
| -------- | ----------------------------------------------------------- |
| feat     | new feature, if not breaking chang equal to MINOR in SemVer |
| fix      | bug fix, equal to PATCH in SemVer                           |
| refactor | refactoring code                                            |
| docs     | documentation change                                        |
| perf     | code improve in term of performance                         |
| test     | add/refactor testing, no production code add                |

### Note about code versioning

Semantic Versioning schema build from there parts: MAJOR.MINOR.PATH

- **PATCH**: is incremented for bug fixes, or other changes that do not change the behavior of the API.
- **MINOR**: backward-compatible changes, not break the API
- **MAJOR**: breaking change , Create new API or add additional features

&nbsp;  
&nbsp;  
&nbsp;

# TAG

- Create / delete

```bash
# Create
git tag -a <tag_name> -m "first tag"
# Delete
$ git tag -d <tag_name>
```



# Tips
## Release notes
- Create commit list between tags

```bash
git log --oneline v0.0.1..v0.0.2
# only subjects
git log --pretty=format:"%s"  v0.0.1..v0.0.2
#
# from tag until current
git log --pretty=format:"%s"  v0.0.1..HEAD
```

- Commit statistic by message type (fix, feat ..)
  
```bash
git log --pretty=format:"%s"  v0.0.1..v0.0.2 \
| cut -d " " -f 1 \
| cut -d "(" -f 1 | cut -d ":" -f 1 | sort -r | uniq -c | sort -nr -k1
# Result
    1 fix
    1 feat
```
