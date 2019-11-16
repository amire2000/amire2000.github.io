---
layout: post
title: vscode and jupyter notebooks 
categories: python
tags: [jupyter, vscode]
image: jupyter.png
public: true
description: Run jupiter notebook from vscode, Demo using matplotlib
---

Microsoft python extension support native editing of Jupyter notebook inside vscode

## notebook file
To Add notebook file, using vscode command palette `Python: Create Blank New Jupyter Notebook`

vscode automatically create Jupyter server locally, setting  remote server uri using command `python: Specify jupyter server uri`

VSCode add few nice future
- Variable explorer
- Plot viewer
- Export as python code
    - good usage in debugger


![](/images/2019-11-15-16-02-19.png)

&nbsp;  
&nbsp;  
&nbsp;  
# magic function

- line function prefix with `%` 
- cell function prefix with `%%`

- `%matplotlib`
- `%matplotlib notebook`
- `%matplotlib inline`: only draw static image in the notebook
  
# opencv
```python
from matplotlib import pyplot as plt
import cv2
img=cv2.imread("opencv.png")
rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
plt.imshow(rgb_img)
plt.show()
```
![](/images/2019-11-16-07-10-20.png)
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Working with Jupyter Notebooks in Visual Studio Code](https://code.visualstudio.com/docs/python/jupyter-support)
- [Working with the Python Interactive window](https://code.visualstudio.com/docs/python/jupyter-support-py)