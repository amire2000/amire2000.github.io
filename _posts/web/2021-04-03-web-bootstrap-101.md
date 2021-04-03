---
layout: post
title: Bootstrap
categories: web
tags: [javascript, ]
description: Using bootstrap
image: 
public: true
---

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta
      name="viewport"
      content="width=device-width, initial-scale=1, shrink-to-fit=no"
    />
    <title>Basic Bootstrap Template</title>
    <!-- Bootstrap CSS file -->
    <link
      rel="stylesheet"
      href="https://stackpath.bootstrapcdn.com/bootstrap/4.5.0/css/bootstrap.min.css"
    />
    <link href="css/bootstrap4-toggle.min.css" rel="stylesheet" />
    <!-- JS files: jQuery first, then Popper.js, then Bootstrap JS -->
    <script src="https://code.jquery.com/jquery-3.5.1.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/popper.js@1.16.0/dist/umd/popper.min.js"></script>
    <script src="https://stackpath.bootstrapcdn.com/bootstrap/4.5.0/js/bootstrap.min.js"></script>
    <script src="scripts/bootstrap4-toggle.min.js"></script>
  </head>
  <body>
    <!-- https://gitbrent.github.io/bootstrap4-toggle/ -->
    <h1>Hello, toggle!</h1>
    <input type="checkbox" checked data-toggle="toggle" data-size="lg" />
    <input type="checkbox" checked data-toggle="toggle" />
    <input type="checkbox" checked data-toggle="toggle" data-size="sm" />
    <input type="checkbox" checked data-toggle="toggle" data-size="xs" />
    <input
      type="checkbox"
      data-toggle="toggle"
      data-on="Enabled"
      data-off="Disabled"
    />
    <input type="range" class="custom-range" id="customRange1">

    
    
  </body>
</html>

```