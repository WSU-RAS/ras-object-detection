ras-object-detection
====================
This is code for training a CNN for object detection for the RAS project using
YOLO on darknet and a few networks with TensorFlow and generating a learning
curve. Look in individual code files for more documentation. Below is the
general process, though you will have to modify for your particular setup.

Also included are the ROS packages to run this on the robot.

## Generating Dataset

### Getting code
Download this repository and all the submodules:

    git clone --recursive https://github.com/floft/ras-object-detection.git
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
    ./tf_gen_config.sh

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
    sbatch yolo_test_terations.srun

    sbatch tf_train.srun
    sbatch tf_eval.srun

After you're done training with TensorFlow, you can export the network:

    sbatch tf_export.srun

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

## Running on ROS

### Jetson Setup

We bought the Nvidia Jetson TX2, [Orbitty carrier
board](http://www.wdlsystems.com/Computer-on-Module/Carrier-Boards/CTI-Orbitty-Carrier-for-NVIDIA-Jetson-TX1.html),
and [Connect Tech active heat
sink](http://www.wdlsystems.com/Computer-on-Module/Cooling-Accessories/CTI-NVIDIA-TX1-Module-Active-Thermal-Heatsink.html).
For the AC power adapter, we used a 12 VDC power supply we had sitting around.

Installing JetPack:

* Install Ubuntu 16.04 in a VM and give it enough ram, maybe 4-6 GiB
  ([src](https://devtalk.nvidia.com/default/topic/988616/jetpack-2-3-1-flash-problems-/?offset=7))
* Enable USB3 in VirtualBox (even though the Jetson is only USB2, if you
  only have USB2 you get a "BootRom is not running" error,
  [src](https://devtalk.nvidia.com/default/topic/1002827/jetson-tx2/problem-flashing-tx-2/2))
* Run JetPack installer and install everything on the host.
* When it gets to the point to install to the Jetson, you'll have to put it
  into the Force Recovery mode (unplug power, plug power, hold in on
  RECOVERY, press Reset, let up on Recovery after 2 seconds). Then NVIDIA
  CORP device shows up in "lsusb." In VirtualBox, under USB (bottom right
  corner) check that for the VM. Press Enter in the VM to say to install.
  Then it'll say it doesn't find the USB device. Check the NVIDIA CORP
  again. Then it'll install.
* After copying over the filesystem, it'll error that it can't find the IP.
  Then you can quit the installer.
* Install the Orbitty modifications to support USB and the PWM fan via
  following [their instructions](http://connecttech.com/resource-center/cti-l4t-nvidia-jetson-board-support-package-release-notes/#bkb-h5).
  You'll end up running the command `sudo ./flash.sh orbitty mmcblk0p1`.
  Note, do this *before* installing anything else since this step will
  overwrite the whole filesystem.
* Plug the Jetson into your host computer ethernet (so you can get IP from
  Wireshark) and share your connection or into a router you can get the Jetson
  IP from. Then run the installer again but make sure to select "no action" to
  install the OS ("Flash OS Image to Target") when re-running. Then when it
  gets to installing other software, e.g. CUDA, it'll ask for the IP, user, and
  pass. Specify the IP and then "nvidia" for both username and password.
  ([src](https://devtalk.nvidia.com/default/topic/1002081/jetson-tx2/jetpack-3-0-install-with-a-vm/))
* Note: after it's all done, to shut down VirtualBox, you probably have to
  uncheck the USB device.

Allow LLMNR so we can resolve "tegra-ubuntu" hostname to SSH into it:

    sudo systemctl enable systemd-resolved
    sudo systemctl start systemd-resolved

Disable the display manager (if desired):

    sudo rm /etc/X11/default-display-manager
    sudo touch /etc/X11/default-display-manager

Set CPUs and GPU to max performance on boot (if desired): add this right before
the `exit 0` in */etc/rc.local* script:

    ( sleep 60 && nvpmodel -m 0 && /home/ubuntu/jetson_clocks.sh )&

Enable universe and multiverse repositories (e.g. to install htop):

    sudo add-apt-repository universe
    sudo add-apt-repository multiverse

Update:

    sudo apt update
    sudo apt upgrade

Install TensorFlow

    sudo apt install python3-pip htop jnettop protobuf-compiler python-pil python-lxml
    sudo -H pip3 install --upgrade pip
    sudo -H pip3 install pillow lxml jupyter matplotlib

If using Python 2.7, this will work:

    git clone https://github.com/peterlee0127/tensorflow-tx2.git
    cd tensorflow-tx2
    sudo apt install python-pip
    sudo -H pip install --upgrade 
    sudo -H pip install tensorflow-1.4.1-cp27-cp27mu-linux_aarch64.whl

If using Python 3.5, as
[described](https://devtalk.nvidia.com/default/topic/1027449/jetson-tx2/run-tensorflow-1-3-on-tx2-stuck/post/5226615/)
you need cuDNNv7. Download the .deb from
[https://developer.nvidia.com/nvidia-tensorrt3rc-download](https://developer.nvidia.com/nvidia-tensorrt3rc-download).
Then:

    sudo dpkg -i nv-tensorrt-repo-ubuntu1604-rc-cuda8.0-trt3.0-20170922_3.0.0-1_arm64.deb
    sudo apt update
    sudo apt install tensorrt python3-dev

Finally, install TensorFlow for Python 3.5:

    git clone https://github.com/jetsonhacks/installTensorFlowJetsonTX.git
    cd installTensorFlowJetsonTX/TX2/
    sudo -H pip3 install tensorflow-1.3.0-cp35-cp35m-linux_aarch64.whl

In your *.ssh/config* for ease of SSHing:

    Host jetson
    HostName tegra-ubuntu
    User nvidia
    ForwardX11 yes
    LocalForward 8899 localhost:8899

For Jupyter, add to your *.bashrc* file:

    alias jn="jupyter-notebook --port=8899"

For TensorFlow Object Detection, add this to the *.bashrc* file:

    export PYTHONPATH=$PYTHONPATH:/home/nvidia/catkin_ws/src/ras-object-detection/models/research/:/home/nvidia/catkin_ws/src/ras-object-detection/models/research/slim/

Copy your model files over to the Jetson *~/networks*:

    scp -r /path/to/ras-object-detection/datasets/SmartHome/*.pb jetson:networks/
    scp /path/to/ras-object-detection/datasets/SmartHome/tf_label_map.pbtxt jetson:networks/
    scp -r /path/to/ras-object-detection/datasets/SmartHome/test_images jetson:networks/

Download this repo. Note you may have some issues if you move this after
initializing the submodules.

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src/
    git clone --recursive https://github.com/floft/ras-object-detection.git

Then, generate the protobuf files:

    cd ~/catkin_ws/src/ras-object-detection/models/research/
    protoc object_detection/protos/*.proto --python_out=.

Install ROS ([src](http://wiki.ros.org/lunar/Installation/Ubuntu)):

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get install ros-lunar-desktop-full
    sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Make ROS work with Python 3:

    sudo apt-get install python3-empy # Errors building messages without this
    sudo -H pip3 install catkin_pkg # Provides rospy for Python 3

Clone the *vision_opencv* package to make *cv_bridge* work with Python 3:

    cd ~/catkin_ws/src
    git clone https://github.com/ros-perception/vision_opencv.git

Now build OpenCV 2 for Python 3
([src](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/)).
Note you'll need roughly 12 GiB of disk space for this. If you don't have that,
then rsync a bunch of your files off the Jetson, delete them, and then copy
them back after you do the `make install`. Also, make sure you run `sudo
~/jetson_clocks.sh` before doing this so it'll compile faster.

    sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
        libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran python3-dev
    wget https://github.com/opencv/opencv/archive/3.3.1.zip -O opencv.zip
    wget https://github.com/opencv/opencv_contrib/archive/3.3.1.zip -O opencv_contrib.zip
    unzip opencv.zip
    unzip opencv_contrib.zip
    cd ~/opencv-3.3.1/
    mkdir build
    cd build
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D INSTALL_C_EXAMPLES=OFF \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.1/modules \
        -D PYTHON_EXECUTABLE=/usr/bin/python3 \
        -D BUILD_EXAMPLES=ON ..
    make -j6
    sudo make install
    sudo ldconfig

Now, if you do "import cv2" it'll still try to use the Python 2.7 version
provided in ROS, which will error. Thus, first try the version we just
installed:

    echo 'export PYTHONPATH="/usr/local/lib/python3.5/dist-packages:$PYTHONPATH"' >> ~/.bashrc

Build everything:

    cd ~/catkin_ws
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3
    catkin_make install

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

### YOLO Setup
For working with YOLO:

    cd ~/catkin_ws/src
    git clone --recursive https://github.com/floft/darknet_ros.git
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_VERSION=3

Copy the final weights over for YOLO into the *darknet_ros* directory:

    scp path/to/ras-object-detection/datasets/SmartHome/backup_100/SmartHome_final.weights \
        jetson:catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/SmartHome.weights
    scp path/to/ras-object-detection/datasets/SmartHome/config.cfg \
        jetson:catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/cfg/SmartHome.cfg
    scp path/to/ras-object-detection/dataset_100.data \
        jetson:catkin_ws/src/darknet_ros/darknet_ros/config/

Create *~/catkin_ws/src/darknet_ros/darknet_ros/config/SmartHome.yaml*,
changing the classes accordingly. Make sure the spaces/tabs are correct or else
it'll error parsing the file. Then change "yolo\_voc.yaml" to "SmartHome.yaml"
in *~/catkin_ws/src/darknet_ros/darknet_ros/launch/darknet_ros.launch*.

    yolo_model:
        config_file:
            name: SmartHome.cfg
        weight_file:
            name: SmartHome.weights
        threshold:
            value: 0.3
        detection_classes:
            names:
              -  food
              -  glass
              -  keys
              -  pillbottle
              -  plant
              -  umbrella
              -  watercan

### Running

#### Speed

If you wish to set the clocks and GPU to full speed, then run
([src](https://devtalk.nvidia.com/default/topic/1018081/jetson-tx2/tensorflow-mobilenet-object-detection-model-in-tx2-is-very-slow-/post/5185487/),
[nvpmodel number reference](http://www.jetsonhacks.com/2017/03/25/nvpmodel-nvidia-jetson-tx2-development-kit/)):

    sudo nvpmodel -m 0 # Maybe?
    sudo ~/jetson_clocks.sh

To check that they're running as fast as possible, check that @1300 is at the
end of the lines printed:

    sudo ~/jetson_clocks.sh --show

#### Running YOLO

    roslaunch darknet_ros darknet_ros.launch

#### Running TensorFlow:

Run the Object Detector after editing the *params.yaml* file:

    roslaunch object_detector_ros object_detector.launch

Or, run components individually:

    roscore
    rosrun object_detector_ros object_detector.py \
        _graph_path:=~/networks/ssd_mobilenet_v1.pb \
        _labels_path:=~/networks/tf_label_map.pbtxt
    rosrun image_view image_view image:=/camera/rgb/image_raw
    rostopic echo /object_detector
