#!/bin/bash
mkdir images && for i in *.mp4; do
    ffmpeg -i "$i" -vf scale=-2:480 tmp.mp4
    ffmpeg -i tmp.mp4 -vf "select=not(mod(n\,10))" -filter:v "crop=640:in_h" -r 1/1 "images/${i//.mp4/} - %03d.png"
    rm tmp.mp4
done
