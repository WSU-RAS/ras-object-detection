#!/bin/bash
ffmpeg -i "2017-10-19 09.54.32.mp4" -vf scale=-2:480 tmp.mp4
#ffmpeg -i tmp.mp4 -filter:v "crop=640:in_h" video_640.mp4
ffmpeg -i tmp.mp4 -vf "select=not(mod(n\,10))" -filter:v "crop=640:in_h" -r 1/1 phone_%03d.png
rm tmp.mp4