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

#
# Learning curve files
#
name="grey_table" # we will use the ${name}.cfg file
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
    # If we killed it before it copied all the files back, do that now
    [[ -n "$backup" && -e "$SCRATCHDIR/backup/" ]] && cp -a "$SCRATCHDIR/backup/"* "$backup/"

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
cp -a "$data/$datasetName" .
echo " - darknet"
cp -ra "$data/darknet" .
echo " - data"
cp -a "$data"/*.txt "$data"/*.data "$data"/*.names "$data"/*.cfg .
echo " - weights"
cp -a "$weights" .
echo "Getting data: done"

echo "Making sure darknet is built: starting"
cd darknet
make -j$SLURM_CPUS_PER_TASK
cd ..
echo "Making sure darknet is built: done"

echo "Extracting data: started"
# Note: only using images/ and labels/ folders, not the other files
tar xzf "$datasetName"
echo "Extracting data: done"

echo "Training network: started"
for i in *.data; do
    # Find the remote backup folder
    backup=$(grep backup "$i" | sed 's/backup = //')

    # Set it to backup locally for speed
    sed -i 's/^backup.*$/backup = backup/g' "$i"
    mkdir -p backup

    if [[ -e "$backup/$name.backup" ]]; then
        if [[ -e "$backup/${name}_final.weights" ]]; then
            # Skip
            echo " - skipping $i since already ran"
        else
            # Continue where we left off
            echo " - continuing training on $i, starting training"
            cp -a "$backup/$name.backup" backup/
            /usr/bin/time -v "$darknet" detector train "$i" "$name.cfg" "backup/$name.backup" -gpus $SLURM_JOB_GPUS
        fi
    else
        # Make remote backup directory
        mkdir -p "$backup"

        # Train and print out stats about time, processor utilization, etc.
        echo " - training on $i, backup $backup"
        /usr/bin/time -v "$darknet" detector train "$i" "$name.cfg" "$weightsName" -gpus $SLURM_JOB_GPUS
    fi

    # If not empty, copy local backup files to the remote backup folder
    [[ -n "$(ls -A backup)" ]] && cp -a backup/* "$backup/"

    rm -rf backup
done
echo "Training network: done"

echo "Deleting workspace: started"
clean_up
echo "Deleting workspace: done"
