
## convert vob to mp4

```
ffmpeg  \
-i <input>.VOB \
-c:v libx264 \
-c:a aac -strict experimental \
<output>.mp4
```


## cut movie part
```
ffmpeg \
-i input.mp4 \
-ss 00:00:30.0 \
-to 00:00:50.0 output.mp4
```
