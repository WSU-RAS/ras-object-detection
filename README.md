ras-object-detection
====================
This is code for training a CNN for object detection for the RAS project using YOLO on darknet and generating a learning curve. Look in individual code files for more documentation. Below is the general process, though you will have to modify for your particular setup.

### Split the data

    ./split_clean.sh
    ./split.py
    wc -l $(ls *.txt | sort --version-sort)

### Copy files over to Kamiak

    ./kamiak_upload.sh

### Compile the modified darknet on Kamiak in an idev session

    ssh kamiak
    idev --gres=gpu:1 # get on a node to build your code (not the login node)
    module load git/2.6.3 gcc/5.2.0 cuda/8.0.44 cudnn/5.1_cuda8.0
    cd /data/vcea/matt.taylor/Projects/ras-yolo/grey-table/darknet
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

    ./kamiak_download.sh
    tail -n 1 $(find results -name '*.txt' | sort --version-sort)
    ./graph.py
