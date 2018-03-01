#!/bin/bash
#
# Convert all to jpg rather than png
#
. /scripts/threading
thread_init

for i in images/*.png; do
    out="$(sed '
    s/\.png/\.jpg/g
    s/images/images_jpg/g
    ' <<< "$i")"
    echo "$out"

    convert -quality 85 "$i" "$out" &
    thread_wait
done

thread_finish
