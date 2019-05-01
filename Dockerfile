FROM starefossen/github-pages
# Copy everything from the current directory into the image
# COPY . /usr/src/app
# Set the locale and override the image settings
ENV LC_ALL C.UTF-8
# Expose port 4000 in this image
EXPOSE 4000/tcp