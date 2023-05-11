# FMAN40

## Overview

This repository contains the code for a project in [FMAN40, Project in Applied Mathematics](https://kurser.lth.se/lot/course/FMAN40) at Lund University. The goal of the project is to create a piecewise planar 3D reconstruction from a set of images.

<!-- TODO: Add some more information here -->

## Dependencies

### macOS

Install [Homebrew](https://brew.sh) then run

```
brew install cmake eigen libjpeg
```

### Ubuntu

```
apt install build-essential cmake libeigen3-dev libjpeg-dev
```

## Building

Setup build folder:
```
mkdir build
cd build
cmake <path to FMAN40 root>
```

Compile:
```
make
```

## Running

First do a sparse reconstruction with [COLMAP](https://colmap.github.io) and export the model as text. Then draw one or more polygons, representing planar surfaces, using the MATLAB script [main.m](Matlab_script/main.m). Finally build and run the `ppr` executable, specifying the directory with the sparse reconstruction, the image directory and the path to the polygon file. The result is a glTF file that can be opened in for example in [this online viewer](https://gltf-viewer.donmccurdy.com) or in [MeshLab](https://www.meshlab.net).

## Datasets

The [datasets](datasets) folder contains a few sample datasets and pre-generated sparse reconstructions and polygon files.

## Miscellaneous

### Code formatting

Use

```
make fmt
```

to format all source files with [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html).

### Documentation

Use

```
make doc
```

to generate documentation with [doxygen](https://www.doxygen.nl). It is saved to the current build directory.

### Sparse reconstruction with COLMAP

scripts/colmap.sh is a script for creating a sparse reconstruction with COLMAP. Usage:

```
CAMERA_MODEL=PINHOLE colmap.sh /path/to/images /path/to/output
```
