#!/bin/bash
# Run sparse reconstruction with COLMAP.
# Usage: colmap.sh <IMAGE_PATH> <OUTPUT_PATH>
# Override COLMAP executable path and camera model with COLMAP_PATH
# and CAMERA_MODEL environment variables, respectively.
set -e
IMAGE_PATH=$1
OUTPUT_PATH=$2
DB_PATH=$OUTPUT_PATH/database.db
mkdir -p $OUTPUT_PATH
COLMAP="${COLMAP_PATH:-colmap}"
CAMERA_MODEL="${CAMERA_MODEL:-PINHOLE}"
$COLMAP feature_extractor --database_path $DB_PATH --image_path $IMAGE_PATH --ImageReader.camera_model $CAMERA_MODEL
$COLMAP exhaustive_matcher --database_path $DB_PATH
$COLMAP mapper --database_path $DB_PATH --image_path $IMAGE_PATH --output_path $OUTPUT_PATH
$COLMAP model_converter --input_path $OUTPUT_PATH/0 --output_path $OUTPUT_PATH --output_type TXT
