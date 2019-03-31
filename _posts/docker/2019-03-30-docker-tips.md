---
layout: post
title: docker tips
categories: docker
tags: [dokcer, tips]
---

# Autocomplete
- Install into `/etc/bash_completion.d/`
```
curl https://raw.githubusercontent.com/docker/docker-ce/master/components/cli/contrib/completion/bash/docker -o /etc/bash_completion.d/docker.sh
```

- Usage
```
#Complete image name
docker images <TAB>
```

