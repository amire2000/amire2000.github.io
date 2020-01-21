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

## Server
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

```html
<html>
   <head>
      <title>AJAX with jQuery and cherrypy</title>
      <script type = "text/javascript" src = "jquery-1.4.2.min.js"></script>
      <script src="gauge.js"></script>
      <script>
          var Gauge = window.Gauge;
          var cpuGauge;

          function render(){
            
        
         cpuGauge = Gauge(document.getElementById("cpuSpeed"), {
          max: 100,
          label: function(value) {
              return Math.round(value) + "/" + this.max;
          },
          value: 50
      });
    };
          
            $(document).ready(function(){
                render();
                $.ajax({
                url: 'm2',
                type: 'get',
                
                success: function(response){
                    cpuGauge.setValue(parseInt(response));
                }
            });
            });
            </script>
		
   </head>
	
   <body>
        <div id="cpuSpeed" class="gauge-container"></div>
   </body>
	
</html>
```
> The dollar sign is commonly used as a shortcut to the function document.getElementById().
# References
- [CherryPy tutorial](https://www.tutorialspoint.com/cherrypy/index.htm)
- [jQuery download](https://blog.jquery.com/2010/02/19/jquery-142-released/)
- [The Dollar Sign ($) and Underscore (_) in JavaScript](https://www.thoughtco.com/and-in-javascript-2037515)
- [svg-gauge](https://github.com/naikus/svg-gauge)