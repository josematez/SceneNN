##!/bin/sh
set -e

# Check if the correct number of arguments is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <scene_id>"
    exit 1
fi

scene_id=$1;

scene_id=$(printf "%03d" "$scene_id")

THIS_FILE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$THIS_FILE_DIR/raw_data/"
EVAL_DIR="$THIS_FILE_DIR/evaluation/"
TARGET_DIR="$ROOT_DIR/$scene_id/"

PLAYBACK_DIR="$THIS_FILE_DIR/utils/playback/playback"
SCENENN_URL="https://hkust-vgd.ust.hk/scenenn/main"
SCENENN_CONTRIB_URL="https://hkust-vgd.ust.hk/scenenn/contrib"


cd "$ROOT_DIR" || exit

mkdir -p "$scene_id"

cd "$TARGET_DIR" || exit

echo "Downloading poses..."
if [ ! -f "$TARGET_DIR/trajectory.log" ]; then
    wget "$SCENENN_URL/$scene_id/trajectory.log"
else
    echo "trajectory.log already exists, skipping download."
fi

echo "Downloading .oni file..."
if [ ! -f "$TARGET_DIR/$scene_id.oni" ]; then
    wget "$SCENENN_URL/oni/$scene_id.oni"
else
    echo "$scene_id.oni already exists, skipping download."
fi

mkdir -p "images"

echo "Extracting RGB-D images..."
$PLAYBACK_DIR "$TARGET_DIR/$scene_id.oni" "$TARGET_DIR/images/"

mkdir -p "intrinsics"

cd "$ROOT_DIR/$scene_id/intrinsics/" || exit
echo "Downloading intrinsics..."
if [ ! -f "$TARGET_DIR/intrinsics/asus.ini" ]; then
    wget "$SCENENN_URL/intrinsic/asus.ini"
else
    echo "asus.ini already exists, skipping download."
fi

cd "$EVAL_DIR" || exit

mkdir -p "$scene_id"

cd "$EVAL_DIR/$scene_id" || exit

echo "Downloading 3D point cloud (.ply file)..."
if [ ! -f "$EVAL_DIR/$scene_id/$scene_id.ply" ]; then
    wget "$SCENENN_URL/$scene_id/$scene_id.ply"
else
    echo "$scene_id.ply already exists, skipping download."
fi

echo "Downloading ground-truth data (.xml file)..."
if [ ! -f "$EVAL_DIR/$scene_id/$scene_id.xml" ]; then
    wget "$SCENENN_CONTRIB_URL/nyu_class/$scene_id/$scene_id.xml"
else
    echo "$scene_id.xml already exists, skipping download."
fi

echo "Generating semantic ground truth..."
python3 $THIS_FILE_DIR/utils/generate_semantic_ground_truth.py \
        $EVAL_DIR/$scene_id/$scene_id.xml \
        -o $EVAL_DIR/$scene_id/semantic_ground_truth.json

echo "Showing semantic ground truth..."
python3 $THIS_FILE_DIR/utils/show_semantic_ground_truth.py \
        -i $EVAL_DIR/$scene_id/semantic_ground_truth.json \
        -o $EVAL_DIR/$scene_id/semantic_ground_truth.png




