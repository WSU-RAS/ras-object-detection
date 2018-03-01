#!/bin/bash
# Run from the ras-object-detection directory
. datasets/SmartHome_more/GarrettTraining/threading
thread_init

for i in datasets/SmartHome_more/GarrettTraining/*.bag; do
    mkdir "${i//.bag/}/"
    python unbagger.py "${i//.bag}/" "$i" &
    thread_wait
done

thread_finish
