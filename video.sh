#!/bin/bash
. config.py
dir="datasets/$dataset"

#
# Generate the frame list for using with ffmpeg
#
genList() {
    local dir="$1"
    local out="$2"

    touch "$out"
    for f in "$dir"/*; do
        echo "file '$f'" >> "$out"
    done
}

genList "$dir/images" "$dir/rgb.list"
#genList "$dir/depth" "$dir/depth.list"

#
# Convert the RGB and depth frames into video files for easy initial viewing
#
# https://en.wikibooks.org/wiki/FFMPEG_An_Intermediate_Guide/image_sequence
[[ ! -e $dir/rgb.mp4 ]] && ffmpeg -pattern_type glob -r 30 -i "$dir/images/*.png" "$dir/rgb.mp4"
#[[ ! -e $dir/depth.mp4 ]] && ffmpeg -pattern_type glob -r 30 -i "$dir/depth/*.png" "$dir/depth.mp4"
