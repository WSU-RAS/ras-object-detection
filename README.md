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

### Images from video
You can record on the robot or use your phone, for instance.

    rosrun image_view video_recorder image:=/camera/rgb/image_raw _filename:=video1.avi

I then extract every 10th frame from all the video files and scale down / crop to 640x480:

    mkdir images && for i in *.mp4; do
        ffmpeg -i "$i" -vf scale=-2:480 tmp.mp4
        ffmpeg -i tmp.mp4 -vf "select=not(mod(n\,10))" -filter:v "crop=640:in_h" -r 1/1 "images/${i//.mp4/} - %03d.png"
        rm tmp.mp4
    done

Then I go through all the images and delete really bad ones, ones without
objects, nearly-identical images, etc. with `ristretto .` that allows easy
deleting without confirmation.

### Images from bag file
To record the camera images in a bag file:

    rosbag record /camera/rgb/image_raw /camera/depth/image_raw

Note, to run this script, you need to be running `roscore`.

    python unbagger.py datasets/NewDataset/ data.bag

### Edit config

Edit `config.py` to set which dataset you wish to work with. Note this is for both Bash and Python, so remember to make syntax work for both.

### Label the images
Output a JSON file with all the images and no annotations yet.

    ./list_images.sh # default: every 10th image
    ./list_images.sh 1 # or: every image
    ./list_images.sh 2 # or: every other image

Open up Sloth (see my Arch [PKGBUILD](https://github.com/floft/PKGBUILDs/tree/master/python-sloth)) and then start to drawing bounding boxes around objects.

    ./annotate.sh

Convert the JSON file to the formats needed for YOLO and TensorFlow.

    ./sloth2yolo.py
    ./yolo_compress_dataset.sh

    ./sloth2tf.py
    ./tf_gen_config.sh rfcn_resnet101 ssd_mobilenet_v1 ssd_inception_v2 faster_rcnn_resnet101

Note: you might want to manually edit the TensorFlow config afterwards to do
different data augmentation, e.g. changing random crops, brightness changes,
etc. If you have huge imbalances in clases, you might want to use JPEG images
rather than PNG since I ended up having the tftrain.record file 53 GiB with PNG
and 25 GiB with JPEG.

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

    sbatch tf_train.srun rfcn_resnet101
    sbatch tf_train.srun ssd_mobilenet_v1
    sbatch tf_train.srun ssd_inception_v2
    sbatch tf_train.srun faster_rcnn_resnet101

    # Note: run eval jobs as soon as the training starts outputting to tflogs/
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

## COCO Dataset (Optional)
If you wish to include some data from the COCO dataset, e.g. I wanted to
include humans, first download the many gigs of files:

    aursync google-cloud-sdk-minimal # For Arch Linux -- install gsutil somehow
    mkdir COCO
    cd COCO
    mkdir annotations
    gsutil -m rsync gs://images.cocodataset.org/annotations annotations
    mkdir train2017
    gsutil -m rsync gs://images.cocodataset.org/train2017 train2017

Extract *annotations_trainval2017.zip*. Then extract all the annotations for
the classes you want (see *coco2sloth.py* script):

    python coco2sloth.py COCO/annotations/annotations_trainval2017/annotations/instances_train2017.json \
        coco_train2017 > output.json

Then copy the images for those annotations to your dataset:

    mkdir -p datasets/SmartHome3/coco_train2017
    grep filename output.json | sort -u | \
        grep -o '\"[^"]*"$' | sed 's/"//g' | \
        sed 's#.*/#../COCO/train2017/#g' | \
        xargs -i cp --reflink=always {} datasets/SmartHome3/coco_train2017/

Combine *ouput.csv* with the *datasets/SmartHome3/sloth.json* manually
annotated file (concatenate the two arrays, i.e. remove the extra "] [" in the
middle of the file).

To see the class imbalance:

    ./balance.sh

On Kamiak, make sure you copy the pretrained weights:

    cd datasets/SmartHome2/
    cp -a faster_rcnn_resnet101_model.ckpt.* rfcn_resnet101_model.ckpt.* \
        ssd_inception_v2_model.ckpt.* ssd_mobilenet_v1_model.ckpt.* ../SmartHome3/
