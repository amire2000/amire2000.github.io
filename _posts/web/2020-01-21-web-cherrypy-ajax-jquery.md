---
layout: post
title: Create UI with cherrypy ajax json and jquery
categories: web
tags: [ajax, cherrypy, jquery]
description: Use cherrypy as web server for ajax UI
public: true
image: jquery_ajax.jpg
---

# Project
```
├── ajax_app.py
└── media
    ├── gauge_demo.html
    ├── gauge.js
    ├── index.html
    ├── jquery-1.4.2.min.js
    └── test.html

```
&nbsp;  
&nbsp;  
&nbsp;  
# jquery
- success
- error
- complete 


```javascript
$.ajax({
    type: "post", url: "/SomeController/SomeAction",
    success: function (data, text) {
        //...
    },
    error: function (request, status, error) {
        alert(request.responseText);
    },
    complete: function() {
      // Schedule the next request when the current one's complete
      setTimeout(worker, 5000);
    }
});
```
&nbsp;  
&nbsp;  
&nbsp;  
# Server

```python
import cherrypy
import os
import simplejson
import sys
import random

MEDIA_DIR = os.path.join(os.path.abspath("."), u"media")


class AjaxApp(object):
    @cherrypy.expose
    def index(self):
        return open(os.path.join(MEDIA_DIR, u'index.html'))

    @cherrypy.expose
    def m1(self):
        cherrypy.response.headers['Content-Type'] = 'application/json'
        data = simplejson.dumps(dict(title="Hello"))
        return data.encode("utf-8")

    @cherrypy.expose
    def m2(self):
        # cherrypy.response.headers['Content-Type'] = 'application/json'
        # data = simplejson.dumps(dict(title="Hello"))
        data = random.randint(0, 100)
        return str(data)

    @cherrypy.expose
    def submit(self, name):
        cherrypy.response.headers['Content-Type'] = 'application/json'
        return simplejson.dumps(dict(title="Hello, %s" % name))


config = {
    "/":
    {
        'tools.encode.on': True,
        'tools.encode.encoding': 'utf-8',
        'tools.staticdir.on': True,
        'tools.staticdir.dir': MEDIA_DIR,
    }
}


def start():
    cherrypy.quickstart(AjaxApp(), '/', config)


if __name__ == "__main__":
    start()

```
&nbsp;  
&nbsp;  
&nbsp;  
# HTML
- refresh page data every one second
- using jquery
- using svg gauge


```html
<html>
  <head>
    <title>AJAX with jQuery and cherrypy</title>
    <script type="text/javascript" src="jquery-1.4.2.min.js"></script>
    <script src="gauge.js"></script>
    <script>
      var Gauge = window.Gauge;
      var cpuGauge;
      //init gauge
      function initGauge() {
        cpuGauge = Gauge(document.getElementById("cpuSpeed"), {
          max: 100,
          label: function(value) {
            return Math.round(value) + "/" + this.max;
          },
          value: 50
        });
      }

      //get data
      function get_data() {
        $.ajax({
          url: "m2",
          type: "get",

          success: function(response) {
            cpuGauge.setValue(parseInt(response));
          }
        });
        setTimeout(get_data, 1000);
      }

      //document ready
      var ONE_SEC = 1000;
      $(document).ready(function() {
        initGauge();
        setTimeout(get_data, ONE_SEC);
      });
    </script>
  </head>

  <body>
    <div id="cpuSpeed" class="gauge-container"></div>
  </body>
</html>

```
> The dollar sign is commonly used as a shortcut to the function document.getElementById().

&nbsp;  
&nbsp;  
&nbsp;  
# References
- [CherryPy tutorial](https://www.tutorialspoint.com/cherrypy/index.htm)
- [jQuery download](https://blog.jquery.com/2010/02/19/jquery-142-released/)
- [The Dollar Sign ($) and Underscore (_) in JavaScript](https://www.thoughtco.com/and-in-javascript-2037515)
- [svg-gauge](https://github.com/naikus/svg-gauge)