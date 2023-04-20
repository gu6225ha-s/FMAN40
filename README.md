# FMAN40

## Overview

TODO

## Dependencies

### macOS

Install [Homebrew](https://brew.sh) then run

```
brew install cmake eigen
```

### Windows

TODO

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

First do a sparse reconstruction with [COLMAP](https://colmap.github.io) and export the model as text. Then draw one or more polygons, representing planar surfaces, using the MATLAB script [main.m](Matlab_script/main.m). Finally build and run the `ppr` executable, specifying the directory with the sparse reconstruction and the path to the polygon file. The result is a glTF file that can be opened in for example in [this viewer](https://gltf-viewer.donmccurdy.com).

## Datasets

The [datasets](datasets) folder contains a few sample datasets and pre-generated sparse reconstructions and polygon files.

## Miscellaneous

### Code formatting

Use

```
make fmt
```

to format all source files with [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html).

### Sparse reconstruction with COLMAP

scripts/colmap.sh is a script for creating a sparse reconstruction with COLMAP. Usage:

```
CAMERA_MODEL=PINHOLE colmap.sh /path/to/images /path/to/output
```
