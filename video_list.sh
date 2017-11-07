#!/bin/bash
#
# Generate the frame list for using with ffmpeg, see video_gen.sh
#
genList() {
    dir="$1"
    out="$2"

    touch "$out"
    for f in "$dir"/*; do
        echo "file '$f'" >> "$out"
    done
}

genList rgb rgb.list
genList depth depth.list
