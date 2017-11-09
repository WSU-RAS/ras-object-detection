ras-object-detection
====================
This is code for training a CNN for object detection for the RAS project using YOLO on darknet and a few networks with TensorFlow and generating a learning curve. Look in individual code files for more documentation. Below is the general process, though you will have to modify for your particular setup.

### Getting code
Download this repository and then update the TensorFlow models submodule:

    git clone https://github.com/floft/ras-object-detection.git
    cd ras-object-detection
    git submodule update

### Capturing bag file of images on robot
Bring everything up on the robot, e.g.:

    roslaunch turtlebot_bringup minimal.launch
    roslaunch turtlebot_bringup 3dsensor.launch
    roslaunch turtlebot_teleop keyboard_teleop.launch

To record the camera video:

    rosrun image_view video_recorder image:=/camera/rgb/image_raw _filename:=video1.avi

To record the camera images in a bag file:

    rosbag record /camera/rgb/image_raw /camera/depth/image_raw

### Unbag the captured footage
Note, to run this script, you need to be running `roscore`.

    python unbagger.py datasets/NewDataset/ data.bag

### Edit config

Edit `config.py` to set which dataset you wish to work with. Note this is for both Bash and Python, so remember to make syntax work for both.

### Label the images
Output a JSON file with all the images and no annotations yet.

    ./list_images.sh

Open up Sloth (see my Arch [PKGBUILD](https://github.com/floft/PKGBUILDs/tree/master/python-sloth)) and then start to drawing bounding boxes around objects.

    ./annotate.sh

Convert the JSON file to the formats for YOLO and TensorFlow.

    ./sloth2yolo.py
    ./sloth2tf.py

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
Example:

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
