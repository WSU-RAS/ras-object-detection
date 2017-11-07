#!/bin/bash
. config.py
sloth --config vision_training_sloth_config.py "datasets/$dataset/sloth.json"
