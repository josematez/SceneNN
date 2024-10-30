                                                                                                                   download.sh                                                                                                                                
##!/bin/sh

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


cd "$ROOT_DIR" || exit

mkdir "$scene_id"

cd "$TARGET_DIR" || exit

echo "Current directory: $TARGET_DIR"

echo "Downloading poses..."
wget "$SCENENN_URL/$scene_id/trajectory.log"

echo "Downloading .oni file..."
wget "$SCENENN_URL/oni/$scene_id.oni"

mkdir "images"

echo "Extracting RGB-D images..."
$PLAYBACK_DIR "$TARGET_DIR/$scene_id.oni" "$TARGET_DIR/images/"

mkdir "intrinsics"

cd "$ROOT_DIR/$scene_id/intrinsics/" || exit
echo "Downloading intrinsics..."
wget "$SCENENN_URL/intrinsic/asus.ini"

cd "$EVAL_DIR" || exit

mkdir "$scene_id"

cd "$EVAL_DIR/$scene_id" || exit

echo "Downloading 3D point cloud (.ply file)..."
wget "$SCENENN_URL/$scene_id/$scene_id.ply"

echo "Downloading ground-truth data (.xml file)..."
wget "$SCENENN_URL/$scene_id/$scene_id.xml"

echo "Sequence $scene_id$ downloaded!"

exit




