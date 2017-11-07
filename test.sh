#!/bin/bash
#SBATCH --job-name=Test
#SBATCH --output=test.out
#SBATCH --error=test.err
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --nodes=1-1
#SBATCH --gres=gpu:tesla:4
#SBATCH --partition=kamiak
#SBATCH --time=0-02:00:00
#SBATCH --mem=5G

. config.py

#
# Learning curve files
#
config="$datasetConfig"
name="$dataset" # we will use the ${name}.cfg file
# .data files, labels, testing.txt, training.txt, etc.
data="$remotedir"
# Contains images/ and labels/
datasetName="$datasetCompressed"
# Darknet executable (in the data folder)
darknet="darknet/darknet"
# Where we'll save the results from testing
resultsFolder="$data/datasets/$dataset/results"

#
# ---
#

export OMP_NUM_THREADS=$SLURM_CPUS_PER_TASK
module load git/2.6.3 gcc/5.2.0 cuda/8.0.44 cudnn/5.1_cuda8.0

function clean_up 
{
    # If we killed it before it copied all the files back, do that now
    [[ -n "$results" && ! -e "$results" && -e "results.txt" ]] && cp -a "results.txt" "${results}.partial"

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
echo " - dataset files"
cp -a "$data"/*.txt "$data"/*.data .
echo " - config file"
cp -a "$data/$config" "$name".cfg
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

echo "Testing network: started"
mkdir -p "$resultsFolder"

# Modify the config file for testing rather than training
#
# Maybe not needed?
sed -i 's/^batch=.*$/batch=1/g; s/^subdivisions=.*$/subdivisions=1/g' "$name.cfg"
# "increases the precision and makes it possible to detect small objects"
# src: https://github.com/AlexeyAB/darknet#how-to-train-to-detect-your-custom-objects
sed -i 's/^width=.*$/width=608/g; s/^height=.*$/height=608/g' "$name.cfg"

for i in *.data; do
    amount=${i//[!0-9]/} # just the number in the filename
    backup=$(grep backup "$i" | sed 's/backup = //')
    final="$backup/${name}_final.weights"

    if [[ -e "$final" ]]; then
        results="$resultsFolder/$amount.txt"

        if [[ -e "$results" ]]; then
            # Skip
            echo " - skipping $i, already tested: $results"
        else
            # Test these final weights
            echo " - testing $final"
            /usr/bin/time -v "$darknet" detector recall "$i" "$name.cfg" "$final" -gpus $SLURM_JOB_GPUS

            # Copy the results back
            echo " - copying results to: $results"
            cp -a "results.txt" "$results"
        fi
    else
        # Skip
        echo " - skipping $i no final weights $final"
    fi
done
echo "Testing network: done"

echo "Deleting workspace: started"
clean_up
echo "Deleting workspace: done"
