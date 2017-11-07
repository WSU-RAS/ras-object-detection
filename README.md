ras-object-detection
====================
This is code for training a CNN for object detection for the RAS project using YOLO on darknet and generating a learning curve. Look in individual code files for more documentation. Below is the general process, though you will have to modify for your particular setup.

First edit `config.py` to set which dataset you wish to work with. Note this is for both Bash and Python.

### Unbag the captured footage

    python unbagger.py datasets/NewDataset/ data.bag

### Label the images
Output a JSON file with all the images and no annotations yet.

    ./list_images.sh

Open up Sloth (see my Arch [PKGBUILD](https://github.com/floft/PKGBUILDs/tree/master/python-sloth)) and then start to drawing bouding boxes around objects. Change the config to set the classes.

    ./annotate.sh

Convert the JSON file to the format YOLO needs.

    ./sloth2yolo.py

### Split the data

    ./split_clean.sh
    ./split.py
    wc -l $(ls *.txt | sort --version-sort)

### Compress the images we'll train with

    ./compress_dataset.sh

### Copy files over to Kamiak

    ./upload.sh

### Compile the modified darknet on Kamiak in an idev session

    ssh kamiak
    idev --gres=gpu:1 # get on a node to build your code (not the login node)
    module load git/2.6.3 gcc/5.2.0 cuda/8.0.44 cudnn/5.1_cuda8.0
    cd /data/vcea/matt.taylor/Projects/ras-object-detection/darknet
    make

### Debugging Darknet

    pushd ../../darknet; make -j4; popd
    gdb -ex run --args ../../darknet/darknet detector recall dataset_1000.data grey_table_testing.cfg backup_1000/grey_table_testing_final.weights

### Train

    sbatch train.sh
    watch -n 1 squeue -A taylor
    tail -f train.out train.err

### Test

    sbatch test.sh
    watch -n 1 squeue -A taylor
    tail -f test.out test.err

### Get results

    ./download.sh
    tail -n 1 $(find results -name '*.txt' | sort --version-sort)
    ./graph.py
