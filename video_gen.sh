#!/bin/bash
#
# Convert the RGB and depth frames into video files for easy initial viewing
#
# https://en.wikibooks.org/wiki/FFMPEG_An_Intermediate_Guide/image_sequence
[[ ! -e rgb.mp4 ]] && ffmpeg -pattern_type glob -r 30 -i "rgb/*.png" rgb.mp4
[[ ! -e depth.mp4 ]] && ffmpeg -pattern_type glob -r 30 -i "depth/*.png" depth.mp4
