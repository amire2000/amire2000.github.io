docker run --rm \
-p 4000:4000 \
-v=$(pwd):/usr/src/app \
-t blog:latest 

exec jekyll serve --host=0.0.0.0

docker run --rm \
-p 4001:4001 \
-v=$(pwd):/usr/src/app \
starefossen/github-pages

docker run --rm \
-p 4000:4000 \
-v=$(pwd)://srv/jekyll \
jekyll/jekyll jekyll build

docker run --rm \
-p 4000:4000 \
-v=$(pwd)://srv/jekyll \
jekyll/jekyll jekyll serve -d _site --watch --force_polling -H 0.0.0.0 -P 4000