#!/bin/bash
#SBATCH --job-name=tftrain
#SBATCH --output=slurm_logs/tf_train.out
#SBATCH --error=slurm_logs/tf_train.err
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --nodes=1-1
#SBATCH --gres=gpu:tesla:8
#SBATCH --partition=taylor
#SBATCH --time=3-00:00:00
#SBATCH --mem=30G

. config.py

#
# ---
#

export OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK
module load git/2.6.3 gcc/5.2.0 cuda/8.0.44 cudnn/5.1_cuda8.0 anaconda3

function clean_up 
{
    # TODO: copy back backup files

    rmworkspace -a -f --name="$SCRATCHDIR"
    exit
}

#Create a scratch workspace
SCRATCHDIR="$(mkworkspace -q -t 7-00:00 -b /local)" # 7 days
trap 'clean_up' EXIT

echo "Scratch space: $SCRATCHDIR"
echo "SLURM_CPUS_PER_TASK: $SLURM_CPUS_PER_TASK"
echo "SLURM_JOB_GPUS: $SLURM_JOB_GPUS"

echo "Getting data: started"
cd "$SCRATCHDIR"
echo " - dataset"
cp -a "$data/$datasetTFtrain" "$data/$datasetTFvalid" "$data/$datasetTFtest" .
echo " - TF models"
cp -ra "$data/models" .
cd models/research/
protoc object_detection/protos/*.proto --python_out=.
cd "$SCRATCHDIR"
echo " - code"
echo "Getting data: done"

echo "Making sure TensorFlow installed: starting"
pip install --user tensorflow-gpu==1.2.1 pillow lxml jupyter matplotlib
echo "Making sure TensorFlow installed: done"

echo "Training network: started"
# TODO: train it on files.record
echo "Training network: done"

echo "Deleting workspace: started"
clean_up
echo "Deleting workspace: done"
