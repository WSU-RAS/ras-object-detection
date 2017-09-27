#!/bin/bash
#SBATCH --job-name=Train
#SBATCH --output=train.out
#SBATCH --error=train.err
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --nodes=1-1
#SBATCH --gres=gpu:tesla:8
#SBATCH --partition=taylor
#SBATCH --time=3-00:00:00
#SBATCH --mem=18G

# Seems like we're using about 5.7 GiB of RAM and each training takes about 6
# hours and 15 minutes. It said we used 567% of 8 CPU's, so probably only need
# 6 CPUs. More GPUs the better, but I don't want to hog them all.


#
# Learning curve files
#
name="grey_table_testing" # we will use the ${name}.cfg file
# .data files, labels, testing.txt, training.txt, etc.
data="/data/vcea/matt.taylor/Projects/ras-yolo/grey-table"
# Contains images/ and labels/
datasetName="TableDarknetDataset.tar.gz"
# Darknet executable (in the data folder)
darknet="darknet/darknet"

#
# Starting weights
#
weightsName="darknet19_448.conv.23"
# Where to copy it from
weights="/data/vcea/matt.taylor/Projects/ras-yolo/darknet/$weightsName"

#
# ---
#

export OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK
module load git/2.6.3 gcc/5.2.0 cuda/8.0.44 cudnn/5.1_cuda8.0

function clean_up 
{
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
cp "$data/$datasetName" .
echo " - darknet"
cp -r "$data/darknet" .
echo " - data"
cp "$data"/*.txt "$data"/*.data "$data"/*.names "$data"/*.cfg .
echo " - weights"
cp "$weights" .
echo "Getting data: done"

echo "Making sure darknet is built: starting"
cd darknet
make
cd ..
echo "Making sure darknet is built: done"

echo "Extracting data: started"
# Note: only using images/ and labels/ folders, not the other files
tar xzf "$datasetName"
echo "Extracting data: done"

echo "Training network: started"
for i in *.data; do
    backup=$(grep backup "$i" | sed 's/backup = //')

    if [[ -e $backup ]]; then
        if [[ -e "$backup/${name}_final.weights" ]]; then
            # Skip
            echo " - skipping $i since already ran"
        else
            # Continue where we left off
            echo " - continuing training on $i, backup $backup"
            /usr/bin/time -v "$darknet" detector train "$i" "$name.cfg" "$backup/$name.backup" -gpus $SLURM_JOB_GPUS
        fi
    else
        # Make backup directory
        mkdir -p "$backup"

        # Train and print out stats about time, processor utilization, etc.
        echo " - training on $i, backup $backup"
        /usr/bin/time -v "$darknet" detector train "$i" "$name.cfg" "$weightsName" -gpus $SLURM_JOB_GPUS
    fi
done
echo "Training network: done"

echo "Deleting workspace: started"
clean_up
echo "Deleting workspace: done"
