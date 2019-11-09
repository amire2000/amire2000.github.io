---
layout: default
permalink: /mini-follow-the-line/
description: mini-follow-the-line
---
<div class="posts">
    {% for post in site.posts %}
    {% if post.categories contains 'mini' %}
    {% include list_ng.html %}
    {% endif %}
    {% endfor %}
</div>