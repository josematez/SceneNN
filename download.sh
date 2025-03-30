##!/bin/sh
set -e

# Check if the correct number of arguments is provided
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Usage: $0 <scene_id> [--evaluation-only]"
    exit 1
fi

scene_id=$1;

# Check if the evaluation-only flag is set
EVALUATION_ONLY=false
if [ "$2" = "--evaluation-only" ]; then
    EVALUATION_ONLY=true
fi

scene_id=$(printf "%03d" "$scene_id")

THIS_FILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAW_DATA_DIR="$THIS_FILE_DIR/raw_data/"
EVALUATION_DIR="$THIS_FILE_DIR/evaluation/"
PLAYBACK_DIR="$THIS_FILE_DIR/utils/playback/playback"

SCENENN_URL="https://hkust-vgd.ust.hk/scenenn/main"
SCENENN_CONTRIB_URL="https://hkust-vgd.ust.hk/scenenn/contrib"

echo "Processing scene: $scene_id"

if [ "$EVALUATION_ONLY" = false ]; then
    # RAW_DATA PART

    cd "$RAW_DATA_DIR" || exit

    mkdir -p "$scene_id"

    cd "$RAW_DATA_DIR/$scene_id/" || exit

    echo "Downloading poses..."
    if [ ! -f "$RAW_DATA_DIR/$scene_id/trajectory.log" ]; then
        wget "$SCENENN_URL/$scene_id/trajectory.log"
    else
        echo "trajectory.log already exists, skipping download."
    fi

    echo "Downloading .oni file..."
    if [ ! -f "$RAW_DATA_DIR/$scene_id/$scene_id.oni" ]; then
        wget "$SCENENN_URL/oni/$scene_id.oni"
    else
        echo "$scene_id.oni already exists, skipping download."
    fi

    mkdir -p "images"

    echo "Extracting RGB-D images..."
    $PLAYBACK_DIR "$RAW_DATA_DIR/$scene_id/$scene_id.oni" "$RAW_DATA_DIR/$scene_id/images/"

    mkdir -p "intrinsics"

    cd "$RAW_DATA_DIR/$scene_id/intrinsics/" || exit
    echo "Downloading intrinsics..."
    if [ ! -f "$RAW_DATA_DIR/$scene_id/intrinsics/asus.ini" ]; then
        wget "$SCENENN_URL/intrinsic/asus.ini"
    else
        echo "asus.ini already exists, skipping download."
    fi
fi

# EVALUATION PART

cd "$EVALUATION_DIR" || exit

mkdir -p "$scene_id"

cd "$EVALUATION_DIR/$scene_id" || exit

echo "Downloading 3D point cloud (.ply file)..."
if [ ! -f "$EVALUATION_DIR/$scene_id/$scene_id.ply" ]; then
    wget "$SCENENN_CONTRIB_URL/nyu_class/$scene_id/$scene_id.ply"
else
    echo "$scene_id.ply already exists, skipping download."
fi

echo "Downloading ground-truth data (.xml file)..."
if [ ! -f "$EVALUATION_DIR/$scene_id/$scene_id.xml" ]; then
    wget "$SCENENN_CONTRIB_URL/nyu_class/$scene_id/$scene_id.xml"
else
    echo "$scene_id.xml already exists, skipping download."
fi

echo "Generating semantic ground truth..."
python3 $THIS_FILE_DIR/utils/generate_semantic_ground_truth.py \
        $EVALUATION_DIR/$scene_id/$scene_id.xml \
        -o $EVALUATION_DIR/$scene_id/semantic_ground_truth.json

echo "Showing semantic ground truth..."
python3 $THIS_FILE_DIR/utils/show_semantic_ground_truth.py \
        -i $EVALUATION_DIR/$scene_id/semantic_ground_truth.json \
        -o $EVALUATION_DIR/$scene_id/semantic_ground_truth.png




