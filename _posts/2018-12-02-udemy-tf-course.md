---
layout: post
title: Udemy tensorflow course
categories: tensorflow
tags: [tensorflow, tf]
---
Learn how to use Google's Deep Learning Framework - TensorFlow with Python! Solve problems with cutting edge techniques!

- [course link](https://www.udemy.com/complete-guide-to-tensorflow-for-deep-learning-with-python/)


## Create docker for the course
~~~docker
FROM continuumio/anaconda3

# add user
RUN useradd -ms /bin/bash user
RUN echo "user   ALL=(ALL:ALL) ALL" >> /etc/sudoers

USER user

WORKDIR /home/user
COPY linux_tfdl_env.yml /tmp
RUN conda env create -f /tmp/linux_tfdl_env.yml

~~~

## Build
```
docker build -t udemy_tf .
```
[linux_tfdl_env.yml](/assets/blog_assets/linux_tfdl_env.yml)

## Run docker
~~~
cd <course_folder>
docker run -it \
    --privileged \
    -v `pwd`:/home/user/notebook \
    -p 8888:8888 \
    --name udemy_tf \
    udemy_tf \
    /bin/bash
~~~

### run notebook
~~~
source activate tfdeeplearning
jupyter notebook --ip='*' --port=8888
~~~