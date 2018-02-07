ras-object-detection
====================
This is code for training a CNN for object detection for the RAS project using
YOLO on darknet and a few networks with TensorFlow and generating a learning
curve. Look in individual code files for more documentation. Below is the
general process, though you will have to modify for your particular setup.

## Generating Dataset

### Getting code
Download this repository and all the submodules:

    git clone --recursive https://github.com/WSU-RAS/ras-object-detection.git
    cd ras-object-detection

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

Convert the JSON file to the formats needed for YOLO and TensorFlow.

    ./sloth2yolo.py
    ./yolo_compress_dataset.sh

    ./sloth2tf.py
    ./tf_gen_config.sh rfcn_resnet101 ssd_mobilenet_v1 ssd_inception_v2 faster_rcnn_resnet101

## Training / Testing
 
### Getting starting weights

For YOLO, [download](https://pjreddie.com/media/files/darknet19_448.conv.23)
and put where you specified in the config file.

For TensorFlow,
[download Faster RCNN](http://storage.googleapis.com/download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz),
extract the model.ckpt.\* files into *datasets/YourDataSet/*. Prepend each
filename with `faster_rcnn_resnet101_`.

For TensorFlow,
[download RFCN](http://download.tensorflow.org/models/object_detection/rfcn_resnet101_coco_2017_11_08.tar.gz),
extract the model.ckpt.\* files into *datasets/YourDataSet/*. Prepend each
filename with `rfcn_resnet101_`.

For TensorFlow,
[download SSD MobileNet](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_2017_11_17.tar.gz),
extract the model.ckpt.\* files into *datasets/YourDataSet/*. Prepend each
filename with `ssd_mobilenet_v1_`.

For TensorFlow,
[download SSD Inception](http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_17.tar.gz),
extract the model.ckpt.\* files into *datasets/YourDataSet/*. Prepend each
filename with `ssd_inception_v2_`.

Before uploading to Kamiak, since *protoc* is not installed, make sure you run:

    cd models/research
    protoc object_detection/protos/*.proto --python_out=.

### Copy files over to Kamiak

    ./kamiak_upload.sh

### Compile the modified darknet on Kamiak in an idev session

    ssh kamiak
    idev --gres=gpu:1 # get on a node to build your code (not the login node)
    module load git/2.6.3 gcc/5.2.0 cuda/8.0.44 cudnn/5.1_cuda8.0
    cd /data/vcea/matt.taylor/Projects/ras-object-detection/darknet
    make

### Training and Testing
Start the train job and then after it has output some weights, you can start
testing what weights it has output.

    sbatch yolo_train.srun
    sbatch yolo_test.srun
    sbatch yolo_test_iterations.srun

    # Note, run eval jobs at the same time as train
    sbatch tf_train.srun rfcn_resnet101
    sbatch tf_train.srun ssd_mobilenet_v1
    sbatch tf_train.srun ssd_inception_v2
    sbatch tf_train.srun faster_rcnn_resnet101
    sbatch tf_eval.srun rfcn_resnet101
    sbatch tf_eval.srun ssd_mobilenet_v1
    sbatch tf_eval.srun ssd_inception_v2
    sbatch tf_eval.srun faster_rcnn_resnet101

After you're done training with TensorFlow, you can export the networks:

    sbatch tf_export.srun rfcn_resnet101
    sbatch tf_export.srun ssd_mobilenet_v1
    sbatch tf_export.srun ssd_inception_v2
    sbatch tf_export.srun faster_rcnn_resnet101

### Monitor progress
YOLO:

    watch -n 1 squeue -A taylor -l
    tail -f slurm_logs/yolo_train.{out,err}

Get YOLO results:

    ./kamiak_download.sh
    ./graph.py

TensorFlow:

    ./kamiak_tflogs.sh # Sync TF log directory every 30 seconds
     tensorboard  --logdir datasets/SmartHome/tflogs
