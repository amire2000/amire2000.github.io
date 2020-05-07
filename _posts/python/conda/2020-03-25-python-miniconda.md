
---
layout: post
title: Miniconda jupyter setup and usage
categories: python
tags: [conda, cling, jupyter]
public: true
description: 
image: miniconda.png
---
# Miniconda

[mini conda installer](https://docs.conda.io/en/latest/miniconda.html#linux-installers)

## Init conda on startup
- Activate/Deactivate shell with conda environment
  
```
conda config --set auto_activate_base false
```

## conda
- List env.
- Add, Activate and Deactivate
  
```
conda env list
conda create -n <env name> python
conda create -n juplab python
conda activate <env name>
conda activate base
conda deactivate

```

# Jupyter Cheat sheet
- Command mode
- Edit mode
  - `ESC` exit to command mode

## Command mode
- A : Insert cell above
- B : Insert cell below
- x,c,v: Cut, Copy, Paste
- p : open command pallette
- Shift + Enter : Run Current cell and insert a new cell below
- Ctrl + Enter: Run current cell
- d d: Delete cell
- 

# Misl
Using miniconda and xeua-cling to run C++ jupyter notebook

```
 conda install xeus-cling notebook -c QuantStack -c conda-forge
```

![](/images/2020-03-26-15-31-12.png)

# Reference
- [Jupyter Notebook Shortcuts](https://towardsdatascience.com/jypyter-notebook-shortcuts-bf0101a98330)