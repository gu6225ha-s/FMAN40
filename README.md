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

## Datasets

TODO

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
